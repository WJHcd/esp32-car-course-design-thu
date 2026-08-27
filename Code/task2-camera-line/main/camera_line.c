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
#include "libuvc/libuvc.h"
#include "libuvc_helper.h"
#include "libuvc_adapter.h"
#include "jpeg_decoder.h"

static const char *TAG = "cam_line";

/* ---------- 缓冲 ---------- */
#define MJPEG_BUF_MAX    (64 * 1024)   /* MJPEG 单帧上限 */
#define RGB_BUF_SIZE     (CAM_FRAME_WIDTH * CAM_FRAME_HEIGHT * 3)
#define FRAME_QUEUE_LEN  2

/* 摄像头筛选（0=不筛选） */
#define CAM_VID 0
#define CAM_PID 0
#define CAM_SERIAL NULL

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

static QueueHandle_t s_frame_queue;
static camera_line_result_t s_result;
static volatile bool s_streaming;

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
    if (xTaskCreate(usb_lib_handler_task, "usb_events", 4096, NULL, 2, NULL) != pdPASS) {
        usb_host_uninstall();
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* ============================================================
 * UVC 帧回调：只拷贝入队，不做耗时处理
 * ============================================================ */
static void frame_callback(uvc_frame_t *frame, void *ptr)
{
    (void)ptr;
    frame_item_t item = {
        .data = heap_caps_malloc(frame->data_bytes, MALLOC_CAP_SPIRAM),
        .len = frame->data_bytes,
    };
    if (item.data == NULL) {
        return;  /* 内存不足直接丢帧 */
    }
    memcpy(item.data, frame->data, frame->data_bytes);
    if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
        /* 队列满：丢最旧帧防积压 */
        frame_item_t old;
        if (xQueueReceive(s_frame_queue, &old, 0) == pdTRUE) {
            heap_caps_free(old.data);
        }
        xQueueSend(s_frame_queue, &item, 0);
    }
}

/* ============================================================
 * 黑线质心检测（RGB888）
 *
 * 策略：只扫描画面下半部分（车头前方地面），逐行统计黑色像素，
 * 对所有黑像素求 X 质心。下半部分避开地平线/远处干扰。
 * ============================================================ */
static void detect_line_centroid(const uint8_t *rgb, camera_line_result_t *out)
{
    const int w = CAM_FRAME_WIDTH;
    const int h = CAM_FRAME_HEIGHT;
    const int y_start = h / 2;          /* 只扫下半幅 */
    const int row_step = 4;             /* 隔行采样，减负 */

    uint32_t black_sum_x = 0;
    uint32_t black_count = 0;
    uint16_t rows_with_line = 0;

    for (int y = y_start; y < h; y += row_step) {
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
        if (row_black >= LINE_MIN_BLACK_PIX / 4) {
            rows_with_line++;
        }
    }

    out->black_rows = rows_with_line;
    if (black_count < LINE_MIN_BLACK_PIX) {
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

        detect_line_centroid(rgb_buf, &s_result);
        s_streaming = true;

        frames++;
        if (esp_timer_get_time() - last_log > 500000) {  /* 500ms 打印一次 */
            last_log = esp_timer_get_time();
            ESP_LOGI(TAG, "帧%" PRIu32 ": %s err=%+.2f rows=%u",
                     frames, s_result.has_line ? "见线" : "丢线",
                     s_result.error, s_result.black_rows);
        }
    }
}

/* ============================================================
 * UVC 初始化 + 流启动（阻塞等待摄像头，放独立任务避免卡 app_main）
 * ============================================================ */
static void uvc_task(void *args)
{
    (void)args;
    uvc_context_t *ctx = NULL;
    uvc_device_t *dev = NULL;
    uvc_device_handle_t *devh = NULL;
    uvc_stream_ctrl_t ctrl;

    ESP_LOGI(TAG, "等待 USB 摄像头连接 ...");
    uvc_error_t res = uvc_init(&ctx, NULL);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_init: %s", uvc_error_string(res));
        vTaskDelete(NULL);
        return;
    }

    while (1) {
        res = uvc_find_device(ctx, &dev, CAM_VID, CAM_PID, CAM_SERIAL);
        if (res >= 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "找到摄像头");

    res = uvc_open(dev, &devh);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_open: %s", uvc_error_string(res));
        vTaskDelete(NULL);
        return;
    }

    res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                          UVC_FRAME_FORMAT_MJPEG,
                                          CAM_FRAME_WIDTH, CAM_FRAME_HEIGHT, CAM_FRAME_FPS);
    if (res < 0) {
        ESP_LOGW(TAG, "320x240@30 协商失败，尝试 640x480@15");
        res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                              UVC_FRAME_FORMAT_MJPEG,
                                              640, 480, 15);
        if (res < 0) {
            ESP_LOGE(TAG, "流协商失败: %s", uvc_error_string(res));
            vTaskDelete(NULL);
            return;
        }
    }

    res = uvc_start_streaming(devh, &ctrl, frame_callback, NULL, 0);
    if (res < 0) {
        ESP_LOGE(TAG, "uvc_start_streaming: %s", uvc_error_string(res));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "摄像头采集已启动");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
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

    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 8192, NULL, 5, NULL, 0) != pdPASS) {
        return false;
    }
    if (xTaskCreate(uvc_task, "uvc", 6144, NULL, 3, NULL) != pdPASS) {
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
