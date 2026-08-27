#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "usb/usb_host.h"
#include "libuvc/libuvc.h"
#include "libuvc_helper.h"
#include "libuvc_adapter.h"
#include "jpeg_decoder.h"
#include "qr_detect.h"
#include "qr_dance.h"
#include "display.h"
#include "motion.h"

/* ============================================================
 * 二维码识别表情+跳舞 主程序
 *
 * 流程：USB UVC 摄像头采集 MJPEG → esp_jpeg 软件解码 RGB888
 *       → 转灰度 → quirc 解码（节流 300ms）→ qr_dance 状态机
 *       → display（TFT 表情）+ motion（转圈/扭动）
 *
 * ⚠️ 调试注意：摄像头占用 GPIO19/20（板载 USB Host），
 *    下载/监视需用外部 USB 转 TTL 接 UART0（GPIO43/44）。
 * ============================================================ */

/* ---------- 运行模式 ----------
 * 0 = 综合：二维码 → 表情 + 跳舞（正式运行）
 * 1 = 纯解码测试（打印二维码文本）
 * 2 = 显示屏测试（轮播表情）
 * 3 = 电机动作测试（转圈/扭动）
 */
#define TEST_MODE 0

static const char *TAG = "qr";

/* ---------- 摄像头流参数（优先 640x480@15，失败降级 320x240@30） ---------- */
#define FRAME_WIDTH   640
#define FRAME_HEIGHT  480
#define FRAME_FPS     15
#define FRAME_FORMAT  UVC_FRAME_FORMAT_MJPEG

#define MJPEG_BUF_SIZE  (128 * 1024)          /* 640x480 MJPEG 单帧上限 */
#define RGB_BUF_SIZE    (FRAME_WIDTH * FRAME_HEIGHT * 3)
#define GRAY_BUF_SIZE   (FRAME_WIDTH * FRAME_HEIGHT)
#define FRAME_QUEUE_LEN 2

/* 摄像头筛选（0 = 不筛选，接任意 UVC 摄像头） */
#define PID 0
#define VID 0
#define SERIAL_NUMBER NULL

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
static SemaphoreHandle_t s_usb_uninstall_ready;

static void usb_lib_handler_task(void *args)
{
    while (1) {
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            usb_host_device_free_all();
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            xSemaphoreGive(s_usb_uninstall_ready);
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
    s_usb_uninstall_ready = xSemaphoreCreateBinary();
    if (s_usb_uninstall_ready == NULL) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }
    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 2, NULL) != pdPASS) {
        vSemaphoreDelete(s_usb_uninstall_ready);
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ============================================================
 * UVC 帧回调：只拷贝入队，不做耗时处理（否则丢帧）
 * ============================================================ */
void frame_callback(uvc_frame_t *frame, void *ptr)
{
    frame_item_t item = {
        .data = heap_caps_malloc(frame->data_bytes, MALLOC_CAP_SPIRAM),
        .len = frame->data_bytes,
    };
    if (item.data == NULL) {
        ESP_LOGE(TAG, "no mem for frame %u bytes", frame->data_bytes);
        return;
    }
    memcpy(item.data, frame->data, frame->data_bytes);
    if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
        /* 队列满：丢弃最旧帧，防止积压 */
        frame_item_t old;
        if (xQueueReceive(s_frame_queue, &old, 0) == pdTRUE) {
            heap_caps_free(old.data);
        }
        xQueueSend(s_frame_queue, &item, 0);
    }
}

/* ============================================================
 * RGB888 → 8bit 灰度（quirc 需要）
 * ============================================================ */
static void rgb888_to_gray(const uint8_t *rgb, uint8_t *gray, int w, int h)
{
    const uint8_t *p = rgb;
    for (int i = 0; i < w * h; i++) {
        gray[i] = (uint8_t)(((uint16_t)p[0] + p[1] + p[2]) / 3);
        p += 3;
    }
}

/* ============================================================
 * 图像处理任务：MJPEG 解码 → 灰度 → QR → 状态机
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
        qr_detect_init(FRAME_WIDTH, FRAME_HEIGHT, 4);
        qr_dance_init();
        ESP_LOGI(TAG, "综合模式：二维码 → 表情 + 跳舞（%dx%d）",
                 FRAME_WIDTH, FRAME_HEIGHT);
    }

    int64_t last_decode = 0;
    int64_t last_print = 0;
    uint32_t frame_count = 0;
    int qr_w = 0, qr_h = 0;

    while (1) {
        frame_item_t item;
        if (xQueueReceive(s_frame_queue, &item, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        /* 1. esp_jpeg 软件解码 MJPEG → RGB888 */
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
        heap_caps_free(item.data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "jpeg decode failed: %s", esp_err_to_name(err));
            continue;
        }

        /* 2. RGB888 → 灰度（用实际解码尺寸，兼容降级分辨率） */
        const int dw = jpeg_out.width;
        const int dh = jpeg_out.height;
        rgb888_to_gray(rgb_buf, gray_buf, dw, dh);

        /* 3. 若实际分辨率变化（协商降级到 320x240），重建 quirc */
        if (dw != qr_w || dh != qr_h) {
            qr_w = dw;
            qr_h = dh;
            qr_detect_init(dw, dh, 4);
        }

        /* 4. 节流解码（quirc 慢，不逐帧解） */
        qr_result_t qr;
        memset(&qr, 0, sizeof(qr));
        const int64_t now = esp_timer_get_time();
        bool fresh = false;
        if (now - last_decode >= DECODE_INTERVAL_US) {
            last_decode = now;
            fresh = qr_detect_process_gray(gray_buf, &qr);
        }

        /* 5. 状态机 / 打印 */
        if (s_dance_mode) {
            qr_dance_update(fresh ? &qr : NULL);
        } else if (fresh) {
            ESP_LOGI(TAG, "QR: \"%s\" (x=%.2f y=%.2f)",
                     qr.text, qr.x_ratio, qr.y_ratio);
        }

        frame_count++;
        if (now - last_print > 500000) {
            last_print = now;
            ESP_LOGI(TAG, "帧%" PRIu32 " 状态=%s",
                     frame_count,
                     s_dance_mode ? qr_dance_state_name() : "TEST");
        }
    }
}

/* ============================================================
 * UVC 采集：初始化 USB Host → 找摄像头 → 协商流 → 开始采集
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

    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 32768, NULL, 5, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "task create failed");
        return;
    }

    /* 等待 USB 摄像头连接 */
    uvc_context_t *ctx = NULL;
    uvc_device_t *dev = NULL;
    uvc_device_handle_t *devh = NULL;
    uvc_stream_ctrl_t ctrl;

    ESP_LOGI(TAG, "等待 USB 摄像头连接 ...");
    uvc_error_t res = uvc_init(&ctx, NULL);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_init: %s", uvc_error_string(res));
        return;
    }

    /* 连接摄像头（VID/PID=0 表示不筛选） */
    while (1) {
        res = uvc_find_device(ctx, &dev, VID, PID, SERIAL_NUMBER);
        if (res >= 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "找到摄像头");

    res = uvc_open(dev, &devh);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_open: %s", uvc_error_string(res));
        return;
    }
    uvc_print_diag(devh, stderr);

    /* 协商流格式（MJPEG 640x480@15；失败则尝试 320x240@30） */
    res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                          UVC_FRAME_FORMAT_MJPEG,
                                          FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS);
    if (res < 0) {
        ESP_LOGW(TAG, "%dx%d@%d 协商失败，尝试 320x240@30",
                 FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS);
        res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                              UVC_FRAME_FORMAT_MJPEG,
                                              320, 240, 30);
        if (res < 0) {
            ESP_LOGE(TAG, "流协商失败: %s", uvc_error_string(res));
            return;
        }
    }
    ESP_LOGI(TAG, "流协商完成，开始采集");

    /* 开始采集（frame_callback 每帧调用） */
    res = uvc_start_streaming(devh, &ctrl, frame_callback, NULL, 0);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_start_streaming: %s", uvc_error_string(res));
        return;
    }

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
#if TEST_MODE == 2
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
    /* 电机动作测试：转圈 / 停 / 反转 / 扭动 */
    motion_init();
    while (1) {
        ESP_LOGI(TAG, "[motion-test] 正转 2s");
        motion_rotate(40);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "[motion-test] 反转 2s");
        motion_rotate(-40);
        vTaskDelay(pdMS_TO_TICKS(2000));
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "[motion-test] 扭动");
        for (int i = 0; i < 6; i++) {
            motion_rotate(30);
            vTaskDelay(pdMS_TO_TICKS(250));
            motion_rotate(-30);
            vTaskDelay(pdMS_TO_TICKS(250));
        }
        motion_stop();
        vTaskDelay(pdMS_TO_TICKS(800));
    }
#else
    /* 0 = 综合（二维码→表情+跳舞）；1 = 纯解码测试 */
    start_camera_pipeline(TEST_MODE == 0);
#endif
}
