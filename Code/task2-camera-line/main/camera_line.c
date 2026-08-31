#include "camera_line.h"
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "jpeg_decoder.h"

static const char *TAG = "cam_line";

/* ---------- 缓冲 ----------
 * 首选 320x240（camera_line.h），摄像头不支持时协商回退到更大分辨率
 * （如 640x480 / 1280x720），因此解码缓冲按 720p 上限分配（仅占 PSRAM） */
#define CAM_MAX_WIDTH     1280
#define CAM_MAX_HEIGHT    720
#define RGB_BUF_SIZE      (CAM_MAX_WIDTH * CAM_MAX_HEIGHT * 3)
#define FRAME_QUEUE_LEN   2

/* uvc_host 驱动缓冲参数 */
#define UVC_FRAME_BUFFERS    3
#define UVC_NUM_URBS         4
#define UVC_URB_SIZE         (10 * 1024)
#define UVC_OPEN_TIMEOUT_MS  5000
#define UVC_OPEN_ATTEMPTS    10

/* MJPEG 帧大小估算（经验：16bpp 原始尺寸 / 6 压缩比，下限 40KB） */
#define MJPEG_SIZE_EST(w, h) (((w) * (h) * 2 / 6) > (40 * 1024) ? ((w) * (h) * 2 / 6) : (40 * 1024))
/* 单帧合法上限，超出直接丢弃 */
#define MJPEG_BUF_MAX        (512 * 1024)

/* ---------- 稳定性判定 ----------
 * 帧率约 10~15fps：连续 N 帧一致才认为状态稳定；
 * 首次见线后再等 SETTLE_MS 让画面/车辆姿态稳定。 */
#define CAM_STABLE_SAMPLES   3     /* 连续 3 帧一致 */
#define CAM_SETTLE_MS        300   /* 见线后稳定等待 300ms */

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

static QueueHandle_t s_frame_queue;
static camera_line_result_t s_result;
static volatile bool s_streaming;          /* 已成功解码出帧 */
static volatile bool s_connected;          /* UVC 流已打开 */
static camera_line_frame_cb_t s_frame_cb;  /* 帧回调（串口/TFT 预览等） */
static volatile bool s_stats_log_enabled = true;

/* ---------- 稳定性判定状态 ---------- */
static uint8_t s_line_run_len;      /* 当前 has_line 状态连续帧数 */
static bool s_stable_state;         /* 已确认的稳定状态 */
static int64_t s_first_seen_ms;     /* 首次见线时刻（用于 settle 等待） */

/* ---------- uvc_host 驱动状态 ---------- */
static uvc_host_stream_hdl_t s_stream;
static uvc_host_stream_config_t s_stream_cfg;   /* 连接事件里填充格式后使用 */
static volatile bool s_connect_pending;         /* 收到 DEVICE_CONNECTED，待开流 */
static volatile bool s_disconnect_pending;      /* 收到 DEVICE_DISCONNECTED，待关流 */
static TaskHandle_t s_uvc_task_handle;

/* ============================================================
 * USB Host 库事件处理
 * ============================================================ */
static void usb_lib_handler_task(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
    }
    vTaskDelete(NULL);
}

static esp_err_t usb_host_init(void)
{
    const usb_host_config_t host_config = {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    esp_err_t err = usb_host_install(&host_config);
    if (err != ESP_OK) {
        return err;
    }
    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 4, NULL) != pdPASS) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ============================================================
 * UVC 帧回调（驱动任务上下文）：
 * 限频拷贝到 PSRAM 入队后立即归还缓冲，不做耗时处理。
 * 640x480 JPEG 解码约 200ms+/帧，全帧率解码会占满单核，
 * 因此采集侧限频到 ~3fps（巡线足够；调 FRAME_PROC_INTERVAL_MS 可改）。
 * ============================================================ */
#define FRAME_PROC_INTERVAL_MS 300

static bool uvc_frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    static int64_t s_last_enqueue_us;
    const int64_t now = esp_timer_get_time();
    if (s_last_enqueue_us != 0 &&
        now - s_last_enqueue_us < (int64_t)FRAME_PROC_INTERVAL_MS * 1000) {
        return true;    /* 限频窗口内直接丢帧 */
    }
    s_last_enqueue_us = now;

    if (frame->data_len == 0 || frame->data_len > MJPEG_BUF_MAX) {
        return true;    /* 异常帧丢弃 */
    }
    frame_item_t item = {
        .data = heap_caps_malloc(frame->data_len, MALLOC_CAP_SPIRAM),
        .len = frame->data_len,
    };
    if (item.data == NULL) {
        return true;    /* 内存不足丢帧 */
    }
    memcpy(item.data, frame->data, frame->data_len);
    if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
        /* 队列满：丢最旧帧防积压 */
        frame_item_t old;
        if (xQueueReceive(s_frame_queue, &old, 0) == pdTRUE) {
            heap_caps_free(old.data);
        }
        xQueueSend(s_frame_queue, &item, 0);
    }
    return true;    /* 数据已拷走，缓冲立即归还驱动 */
}

/* ============================================================
 * 流事件回调（断开/错误不再 abort，2.5.2 行为）
 * ============================================================ */
static void uvc_stream_event_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB 传输错误: %s", esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "摄像头已断开，等待重新接入 ...");
        s_connected = false;
        s_streaming = false;
        s_disconnect_pending = true;
        if (s_uvc_task_handle) {
            xTaskNotifyGive(s_uvc_task_handle);
        }
        break;
    case UVC_HOST_FRAME_BUFFER_OVERFLOW:
        ESP_LOGW(TAG, "帧超出缓冲，已丢弃");
        break;
    case UVC_HOST_FRAME_BUFFER_UNDERFLOW:
        ESP_LOGW(TAG, "无空闲帧缓冲，已丢弃");
        break;
    default:
        break;
    }
}

/* ============================================================
 * 驱动事件回调：设备接入 → 枚举支持格式 → 选定 → 通知开流
 * ============================================================ */
static void uvc_driver_event_cb(const uvc_host_driver_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    if (event->type != UVC_HOST_DRIVER_EVENT_DEVICE_CONNECTED) {
        return;
    }
    const uint8_t addr = event->device_connected.dev_addr;
    const uint8_t index = event->device_connected.uvc_stream_index;
    ESP_LOGI(TAG, "UVC 设备已连接 addr=%u index=%u", addr, index);

    size_t num = event->device_connected.frame_info_num;
    uvc_host_frame_info_t *list = heap_caps_calloc(num, sizeof(uvc_host_frame_info_t), MALLOC_CAP_SPIRAM);
    if (list == NULL ||
        uvc_host_get_frame_list(addr, index, (uvc_host_frame_info_t (*)[])list, &num) != ESP_OK) {
        ESP_LOGE(TAG, "读取摄像头格式列表失败");
        heap_caps_free(list);
        return;
    }

    /* 选格式：优先 320x240，其次 480x320（像素量小、解码省 CPU），
     * 再次 640x480，最后任意 MJPEG */
    int pick_exact = -1, pick_480 = -1, pick_640 = -1, pick_any = -1;
    for (size_t i = 0; i < num; i++) {
        ESP_LOGI(TAG, "  支持: %s %ux%u",
                 list[i].format == UVC_VS_FORMAT_MJPEG ? "MJPEG" : "非MJPEG",
                 list[i].h_res, list[i].v_res);
        if (list[i].format != UVC_VS_FORMAT_MJPEG) {
            continue;
        }
        if (pick_any < 0) {
            pick_any = i;
        }
        if (list[i].h_res == CAM_FRAME_WIDTH && list[i].v_res == CAM_FRAME_HEIGHT && pick_exact < 0) {
            pick_exact = i;
        }
        if (list[i].h_res == 480 && list[i].v_res == 320 && pick_480 < 0) {
            pick_480 = i;
        }
        if (list[i].h_res == 640 && list[i].v_res == 480 && pick_640 < 0) {
            pick_640 = i;
        }
    }
    const int pick = (pick_exact >= 0) ? pick_exact
                   : (pick_480 >= 0) ? pick_480
                   : (pick_640 >= 0) ? pick_640
                   : pick_any;
    if (pick < 0) {
        ESP_LOGE(TAG, "摄像头不支持 MJPEG，无法使用");
        heap_caps_free(list);
        return;
    }
    /* 释放格式表前拷贝选项；释放后继续读 list[pick] 属于未定义行为。 */
    const uint16_t selected_w = list[pick].h_res;
    const uint16_t selected_h = list[pick].v_res;
    ESP_LOGI(TAG, "选定格式: %ux%u MJPEG", selected_w, selected_h);
    heap_caps_free(list);

    s_stream_cfg.usb.dev_addr = addr;
    s_stream_cfg.usb.uvc_stream_index = index;
    s_stream_cfg.usb.vid = UVC_HOST_ANY_VID;
    s_stream_cfg.usb.pid = UVC_HOST_ANY_PID;
    s_stream_cfg.vs_format.h_res = selected_w;
    s_stream_cfg.vs_format.v_res = selected_h;
    s_stream_cfg.vs_format.fps = 0;             /* 设备默认帧率 */
    s_stream_cfg.vs_format.format = UVC_VS_FORMAT_MJPEG;
    s_stream_cfg.advanced.frame_size = MJPEG_SIZE_EST(selected_w, selected_h);

    s_connect_pending = true;
    if (s_uvc_task_handle) {
        xTaskNotifyGive(s_uvc_task_handle);
    }
}

/* ============================================================
 * 黑线质心检测（RGB888）
 *
 * 策略：摄像头俯角可调，调好后黑线位于画面垂直【中部】。
 * 因此只扫描垂直中部窗口（上沿 CAM_SCAN_WIN_TOP_PCT%、
 * 高 CAM_SCAN_WIN_HEIGHT_PCT%）的行，画面上下边缘的
 * 背景/车体不参与判定，避免被误判为线。
 * ============================================================ */
static void detect_line_centroid(const uint8_t *rgb, int w, int h, camera_line_result_t *out)
{
    const int y_start = h * CAM_SCAN_WIN_TOP_PCT / 100;     /* 窗口上沿 */
    const int y_end   = y_start + h * CAM_SCAN_WIN_HEIGHT_PCT / 100;
    const int row_step = 2;             /* 窗口小了，采样密一点 */
    /* 黑像素阈值按实际宽度缩放（阈值按 320 宽标定） */
    const uint32_t min_black = (uint32_t)LINE_MIN_BLACK_PIX * w / CAM_FRAME_WIDTH;

    uint32_t black_sum_x = 0;
    uint32_t black_count = 0;
    uint16_t rows_with_line = 0;
    uint16_t wide_rows = 0;
    const uint32_t wide_row_min = (uint32_t)w * 65u / 100u;

    for (int y = y_start; y < y_end; y += row_step) {
        const uint8_t *row = rgb + (size_t)y * w * 3;
        uint32_t row_black = 0;
        for (int x = 0; x < w; x++) {
            /* RGB888 → 取亮度近似值 */
            const uint8_t r = row[x * 3 + 0];
            const uint8_t g = row[x * 3 + 1];
            const uint8_t b = row[x * 3 + 2];
            const uint8_t lum = (uint8_t)(((uint16_t)r + g + b) / 3);
            if (lum < LINE_BLACK_Y_MAX) {
                black_sum_x += x;
                black_count++;
                row_black++;
            }
        }
        if (row_black >= min_black / 4) {
            rows_with_line++;
        }
        if (row_black >= wide_row_min) {
            wide_rows++;
        }
    }

    out->black_rows = rows_with_line;
    out->wide_black_rows = wide_rows;
    if (black_count < min_black) {
        out->has_line = false;
        out->error = 0.0f;
        return;
    }

    /* 质心归一化到 [-1,+1]：0=画面中心，左负右正 */
    const float cx = (float)black_sum_x / (float)black_count;
    out->error = (cx - (float)(w - 1) / 2.0f) / ((float)(w - 1) / 2.0f);
    out->has_line = true;
}

/* ============================================================
 * 图像处理任务：MJPEG 解码 → 黑线检测
 * ============================================================ */
static void processing_task(void *args)
{
    (void)args;
    uint8_t *rgb_buf = heap_caps_malloc(RGB_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL) {
        ESP_LOGE(TAG, "no mem for rgb buffer %d", RGB_BUF_SIZE);
        vTaskDelete(NULL);
        return;
    }

    int64_t last_log = 0;
    uint32_t frames = 0;

    while (1) {
        frame_item_t item;
        if (xQueueReceive(s_frame_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 解码输出缩放：摄像头分辨率高于 320x240 时按 1/2 输出
         * （480x320→240x160，兼顾解码 CPU 与检测精度）。个别相机
         * 缩放档不支持时逐级回退。 */
        esp_jpeg_image_scale_t try_scales[3] = {
            JPEG_IMAGE_SCALE_1_2, JPEG_IMAGE_SCALE_1_4, JPEG_IMAGE_SCALE_0,
        };
        int n_scales = (s_stream_cfg.vs_format.h_res > CAM_FRAME_WIDTH) ? 3 : 1;
        esp_err_t err = ESP_FAIL;
        esp_jpeg_image_output_t jpeg_out;
        const int64_t t0 = esp_timer_get_time();
        for (int si = 0; si < n_scales && err != ESP_OK; si++) {
            esp_jpeg_image_cfg_t jpeg_cfg = {
                .indata = item.data,
                .indata_size = item.len,
                .outbuf = rgb_buf,
                .outbuf_size = RGB_BUF_SIZE,
                .out_format = JPEG_IMAGE_FORMAT_RGB888,
                .out_scale = try_scales[si],
            };
            err = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
        }
        const int decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        heap_caps_free(item.data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "jpeg decode failed: %s", esp_err_to_name(err));
            continue;
        }
        if (jpeg_out.width > CAM_MAX_WIDTH || jpeg_out.height > CAM_MAX_HEIGHT) {
            ESP_LOGW(TAG, "分辨率 %ux%u 超出解码缓冲，丢弃", jpeg_out.width, jpeg_out.height);
            continue;
        }

        /* 帧回调：把 RGB888 帧转发给外部（串口 ASCII 预览等） */
        if (s_frame_cb) {
            s_frame_cb(rgb_buf, jpeg_out.width, jpeg_out.height);
        }

        detect_line_centroid(rgb_buf, jpeg_out.width, jpeg_out.height, &s_result);

        /*
         * 稳定性判定：has_line 连续 CAM_STABLE_SAMPLES 帧一致才更新
         * s_stable_state；首次见线后记录时刻供 settle 等待。
         */
        {
            static bool last_raw = false;
            if (s_result.has_line == last_raw) {
                if (s_line_run_len < 255) {
                    s_line_run_len++;
                }
            } else {
                last_raw = s_result.has_line;
                s_line_run_len = 1;
            }
            if (s_line_run_len >= CAM_STABLE_SAMPLES &&
                s_stable_state != s_result.has_line) {
                s_stable_state = s_result.has_line;
                if (s_stable_state) {
                    s_first_seen_ms = esp_timer_get_time() / 1000;
                }
            }
        }

        s_result.frame_seq++;
        s_result.stable = (s_line_run_len >= CAM_STABLE_SAMPLES &&
                           s_stable_state == s_result.has_line &&
                           (!s_result.has_line ||
                            (esp_timer_get_time() / 1000 - s_first_seen_ms >= CAM_SETTLE_MS)));
        s_streaming = true;

        frames++;
        /* 解码耗时观测：无论统计日志开关，每 2s 打一条（调参用） */
        {
            static int64_t last_perf_log;
            static uint32_t decode_ms_sum;
            static uint32_t decode_cnt;
            decode_ms_sum += decode_ms;
            decode_cnt++;
            if (esp_timer_get_time() - last_perf_log > 2000000) {
                last_perf_log = esp_timer_get_time();
                ESP_LOGI(TAG, "perf: 解码 %dms/帧, 均值 %" PRIu32 "ms, 已处理 %" PRIu32,
                         decode_ms, decode_ms_sum / decode_cnt, frames);
                decode_ms_sum = 0;
                decode_cnt = 0;
            }
        }
        if (esp_timer_get_time() - last_log > 500000) {  /* 500ms 打印一次 */
            last_log = esp_timer_get_time();
            if (s_stats_log_enabled) {
                ESP_LOGI(TAG, "帧%" PRIu32 ": %s%s err=%+.2f rows=%u wide=%u",
                         frames, s_result.has_line ? "见线" : "丢线",
                         s_result.stable ? "(稳)" : "(抖)",
                         s_result.error, s_result.black_rows, s_result.wide_black_rows);
            }
        }
    }
}

/* ============================================================
 * UVC 流管理任务：等待接入 → 开流 → 等断开 → 关流 → 循环
 * ============================================================ */
static void uvc_task(void *args)
{
    (void)args;
    while (1) {
        /* 等待驱动"设备已连接"事件 */
        while (!s_connect_pending) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        s_connect_pending = false;

        uvc_host_stream_hdl_t stream = NULL;
        bool opened = false;
        for (int attempt = 1; attempt <= UVC_OPEN_ATTEMPTS && !opened; attempt++) {
            ESP_LOGI(TAG, "打开 UVC 流（第 %d/%d 次尝试）...", attempt, UVC_OPEN_ATTEMPTS);
            esp_err_t err = uvc_host_stream_open(&s_stream_cfg,
                                                 pdMS_TO_TICKS(UVC_OPEN_TIMEOUT_MS), &stream);
            if (err == ESP_OK) {
                err = uvc_host_stream_start(stream);
                if (err == ESP_OK) {
                    opened = true;
                    break;
                }
                ESP_LOGE(TAG, "uvc_host_stream_start: %s", esp_err_to_name(err));
                uvc_host_stream_close(stream);
            } else {
                ESP_LOGE(TAG, "uvc_host_stream_open: %s", esp_err_to_name(err));
            }
            vTaskDelay(pdMS_TO_TICKS(2000));
        }

        if (!opened) {
            ESP_LOGE(TAG, "摄像头打开失败，请重新插拔摄像头重试");
            continue;
        }
        s_stream = stream;
        s_connected = true;
        s_streaming = false;
        ESP_LOGI(TAG, "摄像头采集已启动（%ux%u MJPEG）",
                 s_stream_cfg.vs_format.h_res, s_stream_cfg.vs_format.v_res);

        /* 等待断开事件 */
        while (!s_disconnect_pending) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        s_disconnect_pending = false;

        /* 让处理任务消化完队列里的旧帧再关流 */
        vTaskDelay(pdMS_TO_TICKS(300));
        uvc_host_stream_stop(stream);
        esp_err_t err = uvc_host_stream_close(stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uvc_host_stream_close: %s", esp_err_to_name(err));
        }
        s_stream = NULL;
    }
}

bool camera_line_init(void)
{
    memset(&s_result, 0, sizeof(s_result));

    esp_err_t err = usb_host_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return false;
    }

    s_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_item_t));
    if (s_frame_queue == NULL) {
        return false;
    }

    /* uvc_host 驱动（自带后台事件任务） */
    const uvc_host_driver_config_t drv_cfg = {
        .driver_task_stack_size = 4096,
        .driver_task_priority = 6,
        .xCoreID = tskNO_AFFINITY,
        .create_background_task = true,
        .event_cb = uvc_driver_event_cb,
        .user_ctx = NULL,
    };
    err = uvc_host_install(&drv_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uvc_host_install: %s", esp_err_to_name(err));
        return false;
    }

    /* 流配置中固定不变的部分 */
    s_stream_cfg.event_cb = uvc_stream_event_cb;
    s_stream_cfg.frame_cb = uvc_frame_cb;
    s_stream_cfg.user_ctx = NULL;
    s_stream_cfg.advanced.number_of_frame_buffers = UVC_FRAME_BUFFERS;
    s_stream_cfg.advanced.number_of_urbs = UVC_NUM_URBS;
    s_stream_cfg.advanced.urb_size = UVC_URB_SIZE;
    s_stream_cfg.advanced.frame_heap_caps = MALLOC_CAP_SPIRAM;

    /* 解码任务固定在 CPU1，避免挤占 CPU0 上的 USB 主机与主任务 */
    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 10240, NULL, 5, NULL, 1) != pdPASS) {
        return false;
    }
    if (xTaskCreate(uvc_task, "uvc", 6144, NULL, 3, &s_uvc_task_handle) != pdPASS) {
        return false;
    }
    return true;
}

void camera_line_get(camera_line_result_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_result;
}

bool camera_line_ready(void)
{
    return s_streaming;
}

bool camera_line_stable(void)
{
    return s_streaming && s_result.stable;
}

void camera_line_set_frame_cb(camera_line_frame_cb_t cb)
{
    s_frame_cb = cb;
}

void camera_line_set_stats_log(bool enable)
{
    s_stats_log_enabled = enable;
}
