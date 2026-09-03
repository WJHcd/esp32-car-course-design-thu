#include "camera_line.h"
#include "ball_detect.h"
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
static ball_field_result_t s_ball_field;   /* 找球任务三色识别结果（每帧覆盖） */
static volatile bool s_streaming;          /* 已成功解码出帧 */
static volatile bool s_connected;          /* UVC 流已打开 */
static camera_line_frame_cb_t s_frame_cb;  /* 帧回调（串口/TFT 预览等） */
static uint32_t s_frame_cb_fire_count;     /* 诊断：回调触发次数 */
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
 * 因此采集侧限频。当前按 150ms 取最新帧（实测解码 132ms/帧 @240MHz+-O2，
 * 留 ~18ms 空闲喂 CPU1 看门狗），实际帧率约 6.7fps；解码仍是瓶颈。
 * 队列满时仍丢旧帧而不是积压过期画面。
 * ============================================================ */
#define FRAME_PROC_INTERVAL_MS 150

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
 * 黑线检测 · 仿真红外阵列（RGB888 输入）
 *
 * 沿用任务一红外巡线逻辑（infrared.c）：
 *   红外：4 路探头 a/b/c/d（左外→右外），误差 = Σw·hit/count，
 *         w = -3/-1/+1/+3 —— 等价于"命中探头位置的平均值"；
 *         全白 = 丢线，全黑 = T 线/宽线。
 *
 * 相机版：在画面高度 CAM_SCAN_CENTER_FROM_BOTTOM_PCT%（自底向上）
 * 处取一条 CAM_SCAN_BAND_HEIGHT_PCT% 高的水平扫描带（对应红外
 * 阵列安装位置），窗口宽 CAM_SCAN_WIDTH_PCT%、水平居中（对应
 * 阵列物理宽度）。带内像素先 rgb2gray（BT.601 权重）再按
 * CAM_BIN_THRESHOLD 二值化（imbinarize 全局阈值用法），逐列统计：
 * 过半采样行为黑的列视为"虚拟探头命中"。误差 = 命中列在窗口内
 * 的归一化位置均值；a/b/c/d 四等分桶输出 line_pattern 供诊断，
 * 窗口内黑色覆盖 >= CAM_WIDE_ROW_PCT% 的行计入 wide_black_rows
 * （T 线判定不变）。
 * ============================================================ */
#define CAM_SCAN_MAX_WINDOW  (CAM_MAX_WIDTH * CAM_SCAN_WIDTH_PCT / 100 + 1)

static uint16_t s_col_black[CAM_SCAN_MAX_WINDOW];   /* 带内每列判黑行数 */

static void detect_line_centroid(const uint8_t *rgb, int w, int h, camera_line_result_t *out)
{
    /* ---- 窗口几何：水平居中，垂直为扫描带 ---- */
    int win_w = w * CAM_SCAN_WIDTH_PCT / 100;
    if (win_w > w) win_w = w;
    if (win_w < 1) win_w = 1;
    int x_start = (w - win_w) / 2;
    const int x_end = x_start + win_w;
    const int band_half = h * CAM_SCAN_BAND_HEIGHT_PCT / 200;
    const int y_center = h - h * CAM_SCAN_CENTER_FROM_BOTTOM_PCT / 100; /* 自底向上量 */
    int y_top = y_center - band_half;
    int y_bottom = y_center + band_half;
    if (y_top < 0) y_top = 0;
    if (y_bottom > h) y_bottom = h;

    const int sampled_rows = y_bottom - y_top;
    memset(s_col_black, 0, sizeof(s_col_black[0]) * (size_t)win_w);

    uint16_t rows_with_line = 0;
    uint16_t wide_rows = 0;

    /* ---- 扫描带内逐像素 rgb2gray + 二值化，按列/行累计 ---- */
    for (int y = y_top; y < y_bottom; y++) {
        const uint8_t *row = rgb + (size_t)y * w * 3;
        uint32_t row_black = 0;
        for (int x = x_start; x < x_end; x++) {
            const uint8_t r = row[x * 3 + 0];
            const uint8_t g = row[x * 3 + 1];
            const uint8_t b = row[x * 3 + 2];
            /* rgb2gray（BT.601：0.299R+0.587G+0.114B，定点 77/150/29 /256） */
            const uint16_t gray = (uint16_t)((r * 77 + g * 150 + b * 29) >> 8);
            if (gray < CAM_BIN_THRESHOLD) {
                s_col_black[x - x_start]++;
                row_black++;
            }
        }
        if (row_black >= CAM_LINE_MIN_COLUMNS) {
            rows_with_line++;
        }
        if ((uint32_t)row_black * 100u >= (uint32_t)win_w * CAM_WIDE_ROW_PCT) {
            wide_rows++;
        }
    }

    /* ---- 逐列判定虚拟探头命中（过半采样行为黑），求误差与 a/b/c/d ---- */
    const float half_win = (float)(win_w - 1) / 2.0f;
    float sum_norm = 0.0f;
    int hit_cols = 0;
    uint8_t pattern = 0;
    for (int i = 0; i < win_w; i++) {
        if (s_col_black[i] * 2 < sampled_rows) {
            continue;
        }
        hit_cols++;
        sum_norm += ((float)i - half_win) / half_win;
        /* 四等分桶：slot0=最左=a … slot3=最右=d（对应红外位权 -3/-1/+1/+3） */
        int slot = i * 4 / win_w;
        if (slot > 3) slot = 3;
        pattern |= (uint8_t)(1u << (3 - slot));
    }

    out->black_rows = rows_with_line;
    out->wide_black_rows = wide_rows;
    out->line_pattern = pattern;

    if (hit_cols < CAM_LINE_MIN_COLUMNS || sampled_rows <= 0) {
        /* 等价红外全白：丢线 */
        out->has_line = false;
        out->error = 0.0f;
        return;
    }

    /* 命中列归一化位置均值 ∈ [-1,+1]，0=窗口中心（与画面中心重合） */
    out->error = sum_norm / (float)hit_cols;
    out->has_line = true;
}

/* 在 RGB 帧上画 2px 方框（识别结果叠加，随帧送到 TFT 显示） */
static void draw_blob_box(uint8_t *rgb, int w, int h,
                          const ball_blob_t *b,
                          uint8_t cr, uint8_t cg, uint8_t cb)
{
    if (!b->detected) return;
    const int thickness = 4;
    int x0 = (int)b->x_min_px - 3, y0 = (int)b->y_min_px - 3;
    int x1 = x0 + (int)b->width_px + 5, y1 = y0 + (int)b->height_px + 5;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= w) x1 = w - 1;
    if (y1 >= h) y1 = h - 1;
    if (x1 <= x0 || y1 <= y0) return;

    for (int x = x0; x <= x1; x++) {
        for (int k = 0; k < thickness && y0 + k <= y1 - k; k++) {
            uint8_t *pa = rgb + (size_t)(y0 + k) * w * 3 + (size_t)x * 3;
            pa[0] = cr; pa[1] = cg; pa[2] = cb;
            uint8_t *pb = rgb + (size_t)(y1 - k) * w * 3 + (size_t)x * 3;
            pb[0] = cr; pb[1] = cg; pb[2] = cb;
        }
    }
    for (int y = y0; y <= y1; y++) {
        for (int k = 0; k < thickness && x0 + k <= x1 - k; k++) {
            uint8_t *pa = rgb + (size_t)y * w * 3 + (size_t)(x0 + k) * 3;
            pa[0] = cr; pa[1] = cg; pa[2] = cb;
            uint8_t *pb = rgb + (size_t)y * w * 3 + (size_t)(x1 - k) * 3;
            pb[0] = cr; pb[1] = cg; pb[2] = cb;
        }
    }
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
        esp_jpeg_image_scale_t try_scales[3];
        int n_scales;
        if (s_stream_cfg.vs_format.h_res > CAM_FRAME_WIDTH) {
            try_scales[0] = JPEG_IMAGE_SCALE_1_2;
            try_scales[1] = JPEG_IMAGE_SCALE_1_4;
            try_scales[2] = JPEG_IMAGE_SCALE_0;
            n_scales = 3;
        } else {
            /* 原代码 n_scales=1 时仍使用数组首项 1/2，导致相机若恰好
             * 协商到 320x240，实际只解出 160x120，小球尺寸再减半。 */
            try_scales[0] = JPEG_IMAGE_SCALE_0;
            n_scales = 1;
        }
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

#if CAM_FLIP_X
        /* 摄像头水平装反：整帧就地左右镜像。放在检测和回调之前，
         * 巡线误差、ASCII 预览、TFT 预览看到的就是同一张校正后的图。 */
        for (int y = 0; y < jpeg_out.height; y++) {
            uint8_t *row = rgb_buf + (size_t)y * jpeg_out.width * 3;
            for (int x = 0; x < jpeg_out.width / 2; x++) {
                uint8_t *pa = row + (size_t)x * 3;
                uint8_t *pb = row + (size_t)(jpeg_out.width - 1 - x) * 3;
                for (int k = 0; k < 3; k++) {
                    const uint8_t tmp = pa[k];
                    pa[k] = pb[k];
                    pb[k] = tmp;
                }
            }
        }
#endif

        detect_line_centroid(rgb_buf, jpeg_out.width, jpeg_out.height, &s_result);

        /* 找球任务目标识别（红球/蓝球/绿角标），结果供 ball_task 读取。
         * 成本 ~1ms/帧，始终运行；巡线阶段上层不读即可。 */
        static int ball_w = -1, ball_h = -1;
        if (ball_w != (int)jpeg_out.width || ball_h != (int)jpeg_out.height) {
            ball_w = (int)jpeg_out.width;
            ball_h = (int)jpeg_out.height;
            ball_detect_init(ball_w, ball_h);
            ESP_LOGI(TAG, "ball_detect 初始化 %dx%d", ball_w, ball_h);
        }
        ball_detect_process_rgb888(rgb_buf, &s_ball_field);

        /* 识别结果叠加方框：红球黄框 / 蓝球绿框 / 角标紫框（显示色，
         * 实物为绿色——紫框与绿框区分，避免与蓝球的绿框混淆）。
         * 球不用白框，避免包围盒外沿落在白色赛道上后不可见。 */
        draw_blob_box(rgb_buf, jpeg_out.width, jpeg_out.height,
                      &s_ball_field.red_ball, 255, 255, 0);
        draw_blob_box(rgb_buf, jpeg_out.width, jpeg_out.height,
                      &s_ball_field.blue_ball, 0, 255, 0);
        for (int i = 0; i < s_ball_field.marker_count; i++) {
            draw_blob_box(rgb_buf, jpeg_out.width, jpeg_out.height,
                          &s_ball_field.marker[i], 255, 0, 255);
        }

        /* 帧回调挪到识别与叠加之后：TFT 显示的是带识别框的画面 */
        if (s_frame_cb) {
            s_frame_cb_fire_count++;
            if (s_frame_cb_fire_count == 1) {
                ESP_LOGI(TAG, "帧回调首次触发（画面开始送往回调）");
            }
            s_frame_cb(rgb_buf, jpeg_out.width, jpeg_out.height);
        }

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
            ESP_LOGI(TAG, "帧%" PRIu32 ": %s%s err=%+.2f rows=%u wide=%u pat=%c%c%c%c",
                     frames, s_result.has_line ? "见线" : "丢线",
                     s_result.stable ? "(稳)" : "(抖)",
                     s_result.error, s_result.black_rows, s_result.wide_black_rows,
                     (s_result.line_pattern >> 3) & 1 ? '1' : '0',
                     (s_result.line_pattern >> 2) & 1 ? '1' : '0',
                     (s_result.line_pattern >> 1) & 1 ? '1' : '0',
                     s_result.line_pattern & 1 ? '1' : '0');
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

void camera_line_get_ball(ball_field_result_t *out)
{
    if (out == NULL) {
        return;
    }
    *out = s_ball_field;
}

uint32_t camera_line_frame_cb_count(void)
{
    return s_frame_cb_fire_count;
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
