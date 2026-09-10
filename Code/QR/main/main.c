#include <stdio.h>
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
#include "qr_detect.h"
#include "qr_dance.h"
#include "display.h"
#include "motion.h"

/* ============================================================
 * 二维码识别表情+跳舞 主程序
 *
 * 流程：USB UVC 摄像头采集 MJPEG（uvc_host 驱动，接入后枚举
 *       支持格式再选型，兼容实车 JQ-CAM12-720P）
 *       → esp_jpeg 软件解码 RGB888 → 转灰度
 *       → quirc 解码（节流 300ms）→ qr_dance 状态机
 *       → display（TFT 表情）+ motion（转圈/扭动）
 *
 * 摄像头采集层移植自 WJHcd 仓库 task2-camera-line 的 camera_line.c
 * （该库在实车同型号摄像头上验证过；此前 libuvc 固定请求
 *   640x480@15 / 320x240@30 均协商失败 UVC_ERROR_INVALID_MODE）。
 *
 * ⚠️ 调试注意：摄像头占用 GPIO19/20（板载 USB Host），
 *    下载/监视需用外部 USB 转 TTL 接 UART0（GPIO43/44）。
 * ============================================================ */

/* ---------- 运行模式 ----------
 * 0 = 综合：二维码 → 表情 + 跳舞（正式运行）
 * 1 = 纯解码测试（打印二维码文本）
 * 2 = 显示屏测试（轮播表情）
 * 3 = 电机动作测试（转圈/扭动）
 * 4 = GIF 表情动画测试（显示嵌入的 gif_asset.bin）
 */
#define TEST_MODE 4

static const char *TAG = "qr";

/* ---------- 摄像头选型与缓冲 ----------
 * 二维码识别需要足够像素：首选 640x480，不支持时逐级降级；
 * 解码缓冲按 720p 上限分配（仅占 PSRAM），任意选型都能放下。 */
#define CAM_FRAME_WIDTH   640
#define CAM_FRAME_HEIGHT  480
#define CAM_MAX_WIDTH     1280
#define CAM_MAX_HEIGHT    720
#define RGB_BUF_SIZE      (CAM_MAX_WIDTH * CAM_MAX_HEIGHT * 3)
#define GRAY_BUF_SIZE     (CAM_MAX_WIDTH * CAM_MAX_HEIGHT)
#define FRAME_QUEUE_LEN   2

/* uvc_host 驱动缓冲参数（同 task2-camera-line） */
#define UVC_FRAME_BUFFERS    3
#define UVC_NUM_URBS         4
#define UVC_URB_SIZE         (10 * 1024)
#define UVC_OPEN_TIMEOUT_MS  5000
#define UVC_OPEN_ATTEMPTS    10

/* MJPEG 帧大小估算（经验：16bpp 原始尺寸 / 6 压缩比，下限 40KB） */
#define MJPEG_SIZE_EST(w, h) (((w) * (h) * 2 / 6) > (40 * 1024) ? ((w) * (h) * 2 / 6) : (40 * 1024))
/* 单帧合法上限，超出直接丢弃 */
#define MJPEG_BUF_MAX        (512 * 1024)

/* 采集侧限频：回调里只拷贝入队；quirc 慢，全帧率解码会占满单核 */
#define FRAME_ENQUEUE_INTERVAL_MS 150

#define DECODE_INTERVAL_US  300000   /* quirc 解码节流：每 300ms 一次 */

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

static QueueHandle_t s_frame_queue;
static bool s_dance_mode;

/* ============================================================
 * USB Host 库事件处理（官方示例模式）
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
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 4, NULL) != pdPASS) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ============================================================
 * UVC 流状态与回调（驱动任务上下文）
 * ============================================================ */
static uvc_host_stream_hdl_t s_stream;
static uvc_host_stream_config_t s_stream_cfg;   /* 连接事件里填充格式后使用 */
static volatile bool s_connect_pending;         /* 收到 DEVICE_CONNECTED，待开流 */
static volatile bool s_disconnect_pending;      /* 收到 DEVICE_DISCONNECTED，待关流 */
static TaskHandle_t s_uvc_task_handle;

static bool uvc_frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    static int64_t s_last_enqueue_us;
    const int64_t now = esp_timer_get_time();
    if (s_last_enqueue_us != 0 &&
        now - s_last_enqueue_us < (int64_t)FRAME_ENQUEUE_INTERVAL_MS * 1000) {
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
        if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
            heap_caps_free(item.data);
        }
    }
    return true;    /* 数据已拷走，缓冲立即归还驱动 */
}

static void uvc_stream_event_cb(const uvc_host_stream_event_data_t *event, void *user_ctx)
{
    (void)user_ctx;
    switch (event->type) {
    case UVC_HOST_TRANSFER_ERROR:
        ESP_LOGE(TAG, "USB 传输错误: %s", esp_err_to_name(event->transfer_error.error));
        break;
    case UVC_HOST_DEVICE_DISCONNECTED:
        ESP_LOGW(TAG, "摄像头已断开，等待重新接入 ...");
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
 * 选型优先级（二维码需要像素）：640x480 → 480x320 → 320x240 → 任意 MJPEG
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

    int pick_640 = -1, pick_480 = -1, pick_320 = -1, pick_any = -1;
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
        if (list[i].h_res == 640 && list[i].v_res == 480 && pick_640 < 0) {
            pick_640 = i;
        }
        if (list[i].h_res == 480 && list[i].v_res == 320 && pick_480 < 0) {
            pick_480 = i;
        }
        if (list[i].h_res == 320 && list[i].v_res == 240 && pick_320 < 0) {
            pick_320 = i;
        }
    }
    const int pick = (pick_640 >= 0) ? pick_640
                   : (pick_480 >= 0) ? pick_480
                   : (pick_320 >= 0) ? pick_320
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

/* ============================================================
 * RGB888 → 8bit 灰度（quirc 需要）；返回平均亮度（0~255，调试用）
 * ============================================================ */
static uint8_t rgb888_to_gray(const uint8_t *rgb, uint8_t *gray, int w, int h)
{
    const uint8_t *p = rgb;
    uint32_t sum = 0;
    const int n = w * h;
    for (int i = 0; i < n; i++) {
        gray[i] = (uint8_t)(((uint16_t)p[0] + p[1] + p[2]) / 3);
        sum += gray[i];
        p += 3;
    }
    return (uint8_t)(n ? sum / n : 0);
}

/* ============================================================
 * 图像处理任务：MJPEG 解码 → 灰度 → QR → 状态机
 * 固定在 CPU1，避免挤占 CPU0 上的 USB 主机中断与驱动任务。
 * ============================================================ */
static void processing_task(void *args)
{
    uint8_t *rgb_buf = heap_caps_malloc(RGB_BUF_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *gray_buf = heap_caps_malloc(GRAY_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL || gray_buf == NULL) {
        ESP_LOGE(TAG, "no mem for rgb/gray buffer");
        vTaskDelete(NULL);
        return;
    }

    if (s_dance_mode) {
        display_init();
        motion_init();
        qr_dance_init();
        ESP_LOGI(TAG, "综合模式：二维码 → 表情 + 跳舞");
    }

    int64_t last_decode = 0;
    int64_t last_print = 0;
    uint32_t frame_count = 0;
    int qr_w = 0, qr_h = 0;

    while (1) {
        frame_item_t item;
        if (xQueueReceive(s_frame_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            /* 摄像头无新帧（停滞/掉线）时也要推进舞蹈，保证动作到点停车 */
            if (s_dance_mode) {
                qr_dance_update(NULL);
            }
            continue;
        }
        frame_count++;
        const int64_t now = esp_timer_get_time();
        if (now - last_print > 2000000) {
            last_print = now;
            ESP_LOGI(TAG, "帧%" PRIu32 " 状态=%s",
                     frame_count,
                     s_dance_mode ? qr_dance_state_name() : "TEST");
        }

        /* 节流：未到解码点时跳过 JPEG 解码/灰度（quirc 慢，逐帧解纯耗 CPU） */
        if (now - last_decode < DECODE_INTERVAL_US) {
            heap_caps_free(item.data);
            if (s_dance_mode) {
                qr_dance_update(NULL);
            }
            continue;
        }
        last_decode = now;

        /* 1. esp_jpeg 软件解码 MJPEG → RGB888 */
        const int64_t t0 = esp_timer_get_time();
        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = item.data,
            .indata_size = item.len,
            .outbuf = rgb_buf,
            .outbuf_size = RGB_BUF_SIZE,
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            .out_scale = JPEG_IMAGE_SCALE_0,
        };
        esp_jpeg_image_output_t jpeg_out;
        esp_err_t err = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
        const int decode_ms = (int)((esp_timer_get_time() - t0) / 1000);
        heap_caps_free(item.data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "jpeg decode failed: %s", esp_err_to_name(err));
            continue;
        }

        /* 2. RGB888 → 灰度（用实际解码尺寸，兼容任意选型分辨率） */
        const int dw = jpeg_out.width;
        const int dh = jpeg_out.height;
        if (dw <= 0 || dh <= 0 || dw > CAM_MAX_WIDTH || dh > CAM_MAX_HEIGHT) {
            ESP_LOGW(TAG, "解码尺寸异常 %dx%d，跳过", dw, dh);
            continue;
        }
        const uint8_t mean = rgb888_to_gray(rgb_buf, gray_buf, dw, dh);

        /* 3. 若实际分辨率变化（协商选型不同），重建 quirc */
        if (dw != qr_w || dh != qr_h) {
            qr_w = dw;
            qr_h = dh;
            qr_detect_init(dw, dh, 4);
        }

        /* 4. quirc 解码（本帧已通过顶部的节流判断） */
        qr_result_t qr;
        memset(&qr, 0, sizeof(qr));
        const bool fresh = qr_detect_process_gray(gray_buf, &qr);

        /* 5. 状态机 / 打印 */
        if (s_dance_mode) {
            qr_dance_update(fresh ? &qr : NULL);
        } else if (fresh) {
            ESP_LOGI(TAG, "QR: \"%s\" (x=%.2f y=%.2f, 解码 %dms)",
                     qr.text, qr.x_ratio, qr.y_ratio, decode_ms);
        } else {
            /* 调试：每 2s 打一次检测统计（定位角数/解码耗时/画面亮度） */
            static int64_t last_dbg;
            if (now - last_dbg > 2000000) {
                last_dbg = now;
                ESP_LOGI(TAG, "检测: 定位角=%d 解码=%dms 亮度=%d/255 %dx%d",
                         qr.count, decode_ms, mean, dw, dh);
            }
        }

        /* 让出 CPU 给空闲任务：解码循环较重（JPEG+quirc），否则 IDLE1
         * 饿死会触发任务看门狗 */
        vTaskDelay(1);
    }
}

/* ============================================================
 * 摄像头管线启动：USB Host → uvc_host 驱动 → 处理任务 → 流任务
 * ============================================================ */
static void start_camera_pipeline(bool dance_mode)
{
    s_dance_mode = dance_mode;

    ESP_ERROR_CHECK(usb_host_init());

    s_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_item_t));
    if (s_frame_queue == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        return;
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
    esp_err_t err = uvc_host_install(&drv_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uvc_host_install: %s", esp_err_to_name(err));
        return;
    }

    /* 流配置中固定不变的部分 */
    s_stream_cfg.event_cb = uvc_stream_event_cb;
    s_stream_cfg.frame_cb = uvc_frame_cb;
    s_stream_cfg.user_ctx = NULL;
    s_stream_cfg.advanced.number_of_frame_buffers = UVC_FRAME_BUFFERS;
    s_stream_cfg.advanced.number_of_urbs = UVC_NUM_URBS;
    s_stream_cfg.advanced.urb_size = UVC_URB_SIZE;
    s_stream_cfg.advanced.frame_heap_caps = MALLOC_CAP_SPIRAM;

    /* 解码任务固定在 CPU1（栈 32KB：quirc 解码需要），避免挤占 CPU0 的 USB 主机 */
    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 32768, NULL, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return;
    }
    if (xTaskCreate(uvc_task, "uvc", 6144, NULL, 3, &s_uvc_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "uvc task create failed");
        return;
    }

    ESP_LOGI(TAG, "摄像头管线已启动，等待设备接入 ...");
    /* 主线程保持运行 */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* ============================================================
 * 主函数
 * ============================================================ */
void app_main(void)
{
#if TEST_MODE == 4
    /* GIF 表情动画测试：循环播放嵌入的 gif_asset.bin */
    display_init();
    const int n = display_gif_count();
    if (n <= 0) {
        ESP_LOGE(TAG, "GIF 资源无效，退出");
        vTaskDelay(portMAX_DELAY);
    }
    ESP_LOGI(TAG, "GIF 动画测试：%d 帧", n);
    display_idle();   /* 先铺底，动画画在屏中央 128x128 区域 */
    int i = 0;
    while (1) {
        const int delay = display_gif_draw(i);
        /* 推屏耗时已占大头，帧延时压缩到 30ms 提高帧率 */
        vTaskDelay(pdMS_TO_TICKS(delay > 30 ? 30 : delay));
        i = (i + 1) % n;
    }
#elif TEST_MODE == 2
    /* 显示屏测试：轮播表情 */
    display_init();
    while (1) {
        for (int f = 0; f < 5; f++) {
            ESP_LOGI(TAG, "显示表情 face=%d", f);
            display_face(f);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        display_idle();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
#elif TEST_MODE == 3
    /* 电机动作测试：转圈 / 停 / 反转 / 扭动（低速档，便于观察标定） */
    motion_init();
    while (1) {
        ESP_LOGI(TAG, "[motion-test] 正转 2s (20%%)");
        motion_rotate(20);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "[motion-test] 反转 2s (20%%)");
        motion_rotate(-20);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "[motion-test] 扭动 (15%%)");
        for (int i = 0; i < 6; i++) {
            motion_rotate(15);
            vTaskDelay(pdMS_TO_TICKS(500));
            motion_rotate(-15);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));
    }
#else
    /* 0 = 综合（二维码→表情+跳舞）；1 = 纯解码测试 */
    start_camera_pipeline(TEST_MODE == 0);
#endif
}
