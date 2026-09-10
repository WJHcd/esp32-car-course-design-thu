#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

#include "usb/usb_host.h"
#include "usb/uvc_host.h"
#include "jpeg_decoder.h"

#include "qr_detect.h"
#include "camera_task.h"

/* ============================================================
 * UVC 摄像头采集 + 二维码识别（移植自 Code/QR 工程，uvc_host 驱动）
 * 处理任务固定 core0：JPEG 解码 + quirc；语音识别在 core1（voice_cmd）。
 * 识别结果写入 s_last_qr + 时间戳，camera_qr_get() 供攻城槌查询。
 * ============================================================ */

#define TAG "camera"

#define CAM_MAX_WIDTH     1280
#define CAM_MAX_HEIGHT    720
#define RGB_BUF_SIZE      (CAM_MAX_WIDTH * CAM_MAX_HEIGHT * 3)
#define GRAY_BUF_SIZE     (CAM_MAX_WIDTH * CAM_MAX_HEIGHT)
#define FRAME_QUEUE_LEN   2

#define UVC_FRAME_BUFFERS    3
#define UVC_NUM_URBS         4
#define UVC_URB_SIZE         (10 * 1024)
#define UVC_OPEN_TIMEOUT_MS  5000
#define UVC_OPEN_ATTEMPTS    10

#define MJPEG_SIZE_EST(w, h) (((w) * (h) * 2 / 6) > (40 * 1024) ? ((w) * (h) * 2 / 6) : (40 * 1024))
#define MJPEG_BUF_MAX        (512 * 1024)
#define FRAME_ENQUEUE_INTERVAL_MS 150
#define DECODE_INTERVAL_US   300000

typedef struct {
    uint8_t *data;
    size_t len;
} frame_item_t;

static QueueHandle_t s_frame_queue;

/* ---------------- 最近二维码识别结果 ---------------- */
static qr_result_t s_last_qr;
static int64_t s_last_qr_us = -1;

bool camera_qr_get(qr_result_t *out, int64_t *age_ms)
{
    if (s_last_qr_us < 0) {
        return false;
    }
    if (out) {
        *out = s_last_qr;
    }
    if (age_ms) {
        *age_ms = (esp_timer_get_time() - s_last_qr_us) / 1000;
    }
    return true;
}

/* ---------------- USB Host 库事件循环 ---------------- */
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

/* ---------------- UVC 状态 ---------------- */
static uvc_host_stream_hdl_t s_stream;
static uvc_host_stream_config_t s_stream_cfg;
static volatile bool s_connect_pending;
static volatile bool s_disconnect_pending;
static TaskHandle_t s_uvc_task_handle;

/* ---------------- UVC 帧回调：限频拷贝入队 ---------------- */
static bool uvc_frame_cb(const uvc_host_frame_t *frame, void *user_ctx)
{
    (void)user_ctx;
    static int64_t s_last_enqueue_us;
    const int64_t now = esp_timer_get_time();
    if (s_last_enqueue_us != 0 &&
        now - s_last_enqueue_us < (int64_t)FRAME_ENQUEUE_INTERVAL_MS * 1000) {
        return true;
    }
    s_last_enqueue_us = now;

    if (frame->data_len == 0 || frame->data_len > MJPEG_BUF_MAX) {
        return true;
    }
    frame_item_t item = {
        .data = heap_caps_malloc(frame->data_len, MALLOC_CAP_SPIRAM),
        .len = frame->data_len,
    };
    if (item.data == NULL) {
        return true;
    }
    memcpy(item.data, frame->data, frame->data_len);
    if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
        frame_item_t old;
        if (xQueueReceive(s_frame_queue, &old, 0) == pdTRUE) {
            heap_caps_free(old.data);
        }
        if (xQueueSend(s_frame_queue, &item, 0) != pdTRUE) {
            heap_caps_free(item.data);
        }
    }
    return true;
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

/* ---------------- 接入后枚举格式并选型（二维码优先 640x480） ---------------- */
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

    /* 同分辨率多档帧率时选 default_interval 最大（=最低帧率）：给同线的
     * UAC 音频让出 FS USB 带宽，避免视频帧损坏。 */
    int pick_640 = -1, pick_480 = -1, pick_320 = -1, pick_any = -1;
    uint32_t iv_640 = 0, iv_480 = 0, iv_320 = 0, iv_any = 0;
    for (size_t i = 0; i < num; i++) {
        ESP_LOGI(TAG, "  支持: %s %ux%u @%.1ffps",
                 list[i].format == UVC_VS_FORMAT_MJPEG ? "MJPEG" : "非MJPEG",
                 list[i].h_res, list[i].v_res,
                 list[i].default_interval ? 1e7 / (double)list[i].default_interval : 0.0);
        if (list[i].format != UVC_VS_FORMAT_MJPEG) {
            continue;
        }
        const uint32_t iv = list[i].default_interval;
        if (pick_any < 0 || iv > iv_any) {
            pick_any = i;
            iv_any = iv;
        }
        if (list[i].h_res == 640 && list[i].v_res == 480 && iv > iv_640) {
            pick_640 = i;
            iv_640 = iv;
        }
        if (list[i].h_res == 480 && list[i].v_res == 320 && iv > iv_480) {
            pick_480 = i;
            iv_480 = iv;
        }
        if (list[i].h_res == 320 && list[i].v_res == 240 && iv > iv_320) {
            pick_320 = i;
            iv_320 = iv;
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
    const uint16_t selected_w = list[pick].h_res;
    const uint16_t selected_h = list[pick].v_res;
    /* ⚠️ FS USB 总带宽紧张（UVC+UAC 麦克风+喇叭同线）：按描述符里最低
     * 帧率开流（必须与设备支持的档位精确匹配，否则 ESP_ERR_NOT_FOUND）。 */
    const uint32_t picked_iv = (pick == pick_640) ? iv_640
                             : (pick == pick_480) ? iv_480
                             : (pick == pick_320) ? iv_320
                             : iv_any;
    const float picked_fps = picked_iv ? (float)(1e7 / (double)picked_iv) : 0.0f;

    ESP_LOGI(TAG, "选定格式: %ux%u MJPEG @%.1ffps", selected_w, selected_h, picked_fps);
    heap_caps_free(list);

    s_stream_cfg.usb.dev_addr = addr;
    s_stream_cfg.usb.uvc_stream_index = index;
    s_stream_cfg.usb.vid = UVC_HOST_ANY_VID;
    s_stream_cfg.usb.pid = UVC_HOST_ANY_PID;
    s_stream_cfg.vs_format.h_res = selected_w;
    s_stream_cfg.vs_format.v_res = selected_h;
    s_stream_cfg.vs_format.fps = picked_fps;   /* 描述符最低帧率（本机=25fps） */
    s_stream_cfg.vs_format.format = UVC_VS_FORMAT_MJPEG;
    s_stream_cfg.advanced.frame_size = MJPEG_SIZE_EST(selected_w, selected_h);

    s_connect_pending = true;
    if (s_uvc_task_handle) {
        xTaskNotifyGive(s_uvc_task_handle);
    }
}

static void uvc_task(void *args)
{
    (void)args;
    while (1) {
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

        while (!s_disconnect_pending) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
        s_disconnect_pending = false;

        vTaskDelay(pdMS_TO_TICKS(300));
        uvc_host_stream_stop(stream);
        esp_err_t err = uvc_host_stream_close(stream);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "uvc_host_stream_close: %s", esp_err_to_name(err));
        }
        s_stream = NULL;
    }
}

/* ---------------- RGB888 → 灰度 ---------------- */
static void rgb888_to_gray(const uint8_t *rgb, uint8_t *gray, int w, int h)
{
    const uint8_t *p = rgb;
    for (int i = 0; i < w * h; i++) {
        gray[i] = (uint8_t)(((uint16_t)p[0] + p[1] + p[2]) / 3);
        p += 3;
    }
}

/* ---------------- 图像处理任务：解码 → 灰度 → quirc → 共享结果 ---------------- */
static void processing_task(void *args)
{
    (void)args;
    uint8_t *rgb_buf = heap_caps_malloc(RGB_BUF_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *gray_buf = heap_caps_malloc(GRAY_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (rgb_buf == NULL || gray_buf == NULL) {
        ESP_LOGE(TAG, "no mem for rgb/gray buffer");
        vTaskDelete(NULL);
        return;
    }

    /* quirc 只看画面中央裁剪窗：复杂背景下整帧洪水填充可超 5s（WDT 饿死），
     * 裁剪后像素量 -36%，且攻城槌瞄准时目标必然在中央。 */
    const int CROP_W = 256, CROP_H = 192;
    uint8_t *gray_crop = heap_caps_malloc(CROP_W * CROP_H, MALLOC_CAP_SPIRAM);
    if (gray_crop == NULL) {
        ESP_LOGE(TAG, "no mem for gray crop");
        vTaskDelete(NULL);
        return;
    }

    int64_t last_decode = 0;
    int qr_w = 0, qr_h = 0;

    while (1) {
        frame_item_t item;
        if (xQueueReceive(s_frame_queue, &item, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        if (esp_timer_get_time() - last_decode < DECODE_INTERVAL_US) {
            heap_caps_free(item.data);
            continue;
        }
        last_decode = esp_timer_get_time();

        {
            static bool first_frame_logged;
            if (!first_frame_logged) {
                first_frame_logged = true;
                ESP_LOGW(TAG, "首帧 %u 字节，开始 JPEG 解码", (unsigned)item.len);
            }
        }

        esp_jpeg_image_cfg_t jpeg_cfg = {
            .indata = item.data,
            .indata_size = item.len,
            .outbuf = rgb_buf,
            .outbuf_size = RGB_BUF_SIZE,
            .out_format = JPEG_IMAGE_FORMAT_RGB888,
            /* ⚠️ 半分辨率（320x240）：ESP-SR 占用总线后，quirc 在 PSRAM 上
             *   640x480 单次解码可达 20s+（喂不饱看门狗）；减 4 倍工作量。
             *   二维码在 0.5~1m 内占画面比例足够，识别不受影响。 */
            .out_scale = JPEG_IMAGE_SCALE_1_2,
        };
        const int64_t t0 = esp_timer_get_time();
        esp_jpeg_image_output_t jpeg_out;
        esp_err_t err = esp_jpeg_decode(&jpeg_cfg, &jpeg_out);
        const int jpeg_ms = (int)((esp_timer_get_time() - t0) / 1000);
        heap_caps_free(item.data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "jpeg decode failed: %s", esp_err_to_name(err));
            continue;
        }
        vTaskDelay(1);   /* JPEG 与 quirc 之间让出一次 */

        const int dw = jpeg_out.width;
        const int dh = jpeg_out.height;
        if (dw <= 0 || dh <= 0 || dw > CAM_MAX_WIDTH || dh > CAM_MAX_HEIGHT) {
            ESP_LOGW(TAG, "解码尺寸异常 %dx%d，跳过", dw, dh);
            continue;
        }
        rgb888_to_gray(rgb_buf, gray_buf, dw, dh);

        /* 中央裁剪窗（越界自动收缩） */
        const int cw = dw < CROP_W ? dw : CROP_W;
        const int ch = dh < CROP_H ? dh : CROP_H;
        const int ox = (dw - cw) / 2, oy = (dh - ch) / 2;
        for (int y = 0; y < ch; y++) {
            memcpy(gray_crop + (size_t)y * cw,
                   gray_buf + (size_t)(y + oy) * dw + ox, cw);
        }

        if (cw != qr_w || ch != qr_h) {
            qr_w = cw;
            qr_h = ch;
            qr_detect_init(cw, ch, 4);
        }

        qr_result_t qr;
        memset(&qr, 0, sizeof(qr));
        const int64_t t1 = esp_timer_get_time();
        const bool got = qr_detect_process_gray(gray_crop, &qr);
        const int quirc_ms = (int)((esp_timer_get_time() - t1) / 1000);
        if (got && qr.detected) {
            s_last_qr = qr;
            s_last_qr_us = esp_timer_get_time();
            ESP_LOGI(TAG, "QR: \"%s\" (x=%.2f y=%.2f)", qr.text, qr.x_ratio, qr.y_ratio);
        }

        /* 耗时观测：每 3s 打一次（含定位角计数，判断二维码可见性） */
        static int64_t last_perf;
        {
            static int jpeg_acc, quirc_acc, cnt, cap_acc;
            jpeg_acc += jpeg_ms;
            quirc_acc += quirc_ms;
            cap_acc += qr.count;
            cnt++;
            if (esp_timer_get_time() - last_perf > 3000000) {
                last_perf = esp_timer_get_time();
                ESP_LOGI(TAG, "perf: jpeg=%dms quirc=%dms 定位角=%d.%d/帧（%d 帧）",
                         jpeg_ms, quirc_ms, cap_acc / cnt, cap_acc % cnt, cnt);
                jpeg_acc = quirc_acc = cap_acc = cnt = 0;
            }
        }

        vTaskDelay(1);   /* 喂 IDLE，避免 CPU0 看门狗 */
    }
}

esp_err_t camera_task_start(void)
{
    /* 1. usb host（本工程唯一的 install 点；UAC 模块复用） */
    esp_err_t err = usb_host_install(&(const usb_host_config_t) {
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    });
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return err;
    }
    static bool lib_task_started;
    if (!lib_task_started) {
        if (xTaskCreatePinnedToCore(usb_lib_handler_task, "usb_lib", 4096, NULL, 10, NULL, 0) != pdPASS) {
            return ESP_ERR_NO_MEM;
        }
        lib_task_started = true;
    }

    /* 2. 帧队列 + uvc 驱动 */
    s_frame_queue = xQueueCreate(FRAME_QUEUE_LEN, sizeof(frame_item_t));
    if (s_frame_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

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
        return err;
    }

    s_stream_cfg.event_cb = uvc_stream_event_cb;
    s_stream_cfg.frame_cb = uvc_frame_cb;
    s_stream_cfg.user_ctx = NULL;
    s_stream_cfg.advanced.number_of_frame_buffers = UVC_FRAME_BUFFERS;
    s_stream_cfg.advanced.number_of_urbs = UVC_NUM_URBS;
    s_stream_cfg.advanced.urb_size = UVC_URB_SIZE;
    s_stream_cfg.advanced.frame_heap_caps = MALLOC_CAP_SPIRAM;

    /* 3. 图像处理任务（core0）+ UVC 流管理任务 */
    if (xTaskCreatePinnedToCore(processing_task, "img_proc", 32768, NULL, 5, NULL, 0) != pdPASS ||
        xTaskCreate(uvc_task, "uvc", 6144, NULL, 3, &s_uvc_task_handle) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "摄像头管线已启动，等待设备接入 ...");
    return ESP_OK;
}
