/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
/* ============================================================
 * 摄像头找球主程序（科目三核心）
 *
 * 流程：USB UVC 摄像头采集 MJPEG → esp_jpeg 软件解码 RGB888
 *       → ball_detect 颜色识别（红/白球）
 *       → find_ball 状态机（SEARCH/TRACK/ALIGN/APPROACH/KICK）
 *
 * 视觉结果：ball_detect_result_t（detected/color/x_ratio/y_ratio/area_ratio）
 * 控制输出：gimbal（云台 pan/tilt）、motion（电机，暂定接口）、kick（踢球）
 *
 * ⚠️ 调试注意：摄像头占用 GPIO19/20（板载 USB Host），
 *    下载/监视需用外部 USB 转 TTL 接 UART0（GPIO43/44）。
 *
 * ⚠️ 引脚待定：云台/踢球舵机引脚当前未接线，gimbal/kick 为占位实现
 *     （见 gimbal.h / kick.h / motion.h）。
 * ============================================================ */
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
#include "ball_detect.h"
#include "find_ball.h"

static const char *TAG = "camera";

/* ---------- 云台/踢球舵机 GPIO（暂定，0=未接线，确定后修改） ---------- */
#define GIMBAL_PAN_PIN   0
#define GIMBAL_TILT_PIN  0
#define KICK_PIN         0

/* ---------- 摄像头流参数（ESP32-S3 USB Full Speed 限制） ---------- */
#define FRAME_WIDTH   320
#define FRAME_HEIGHT  240
#define FRAME_FPS     30
#define FRAME_FORMAT  UVC_FRAME_FORMAT_MJPEG

/* ---------- 缓冲大小 ---------- */
#define MJPEG_BUF_SIZE   (64 * 1024)          /* MJPEG 单帧上限 */
#define RGB_BUF_SIZE     (FRAME_WIDTH * FRAME_HEIGHT * 3)
#define FRAME_QUEUE_LEN  2                    /* 帧队列深度 */

/* 摄像头筛选（0 = 不筛选，接任意 UVC 摄像头） */
#define PID 0
#define VID 0
#define SERIAL_NUMBER NULL

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

static QueueHandle_t s_frame_queue;

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
 * 图像处理任务：MJPEG 解码 → 颜色识别 → 打印方位
 * ============================================================ */
static void processing_task(void *args)
{
    uint8_t *rgb_buf = heap_caps_malloc(RGB_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL) {
        ESP_LOGE(TAG, "no mem for rgb buffer %d", RGB_BUF_SIZE);
        vTaskDelete(NULL);
        return;
    }

    ball_detect_init(FRAME_WIDTH, FRAME_HEIGHT);
    find_ball_init(GIMBAL_PAN_PIN, GIMBAL_TILT_PIN, KICK_PIN);

    ESP_LOGI(TAG, "找球处理任务启动: %dx%d RGB888 (esp_jpeg 软件解码)", FRAME_WIDTH, FRAME_HEIGHT);

    int64_t last_print = 0;
    uint32_t frame_count = 0;

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

        /* 2. 颜色识别找球 */
        ball_detect_result_t res;
        ball_detect_process_rgb888(rgb_buf, &res);

        /* 3. 状态机：云台跟踪/电机接近/踢球 */
        find_ball_update(&res);

        /* 4. 打印（每 200ms 一次，避免刷屏） */
        frame_count++;
        if (esp_timer_get_time() - last_print > 200000) {   /* 200ms */
            last_print = esp_timer_get_time();
            if (res.detected) {
                const char *cname = (res.color == BALL_COLOR_RED) ? "红" : "白";
                ESP_LOGI(TAG, "帧%" PRIu32 ": %s 检测到%s球 x=%.2f y=%.2f area=%.3f pan=%d",
                         frame_count, find_ball_state_name(find_ball_get_state()),
                         cname, res.x_ratio, res.y_ratio, res.area_ratio,
                         /* gimbal 角度暂不打印（引脚未接）占位 */
                         0);
            } else {
                ESP_LOGI(TAG, "帧%" PRIu32 ": %s 未检测到球",
                         frame_count, find_ball_state_name(find_ball_get_state()));
            }
        }
    }
}

/* ============================================================
 * 主函数：UVC 初始化 + 采集
 * ============================================================ */
void app_main(void)
{
    ESP_ERROR_CHECK(usb_host_init());

    s_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_item_t));
    if (s_frame_queue == NULL) {
        ESP_LOGE(TAG, "queue create failed");
        return;
    }

    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 8192, NULL, 5, NULL, 0) != pdPASS) {
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

    /* 协商流格式（MJPEG 320x240@30fps；失败则尝试 640x480@15） */
    res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                          UVC_FRAME_FORMAT_MJPEG,
                                          FRAME_WIDTH, FRAME_HEIGHT, FRAME_FPS);
    if (res < 0) {
        ESP_LOGW(TAG, "320x240@30 协商失败，尝试 640x480@15");
        res = uvc_get_stream_ctrl_format_size(devh, &ctrl,
                                              UVC_FRAME_FORMAT_MJPEG,
                                              640, 480, 15);
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
