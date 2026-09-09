#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/uac_host.h"

#include "audio_in.h"

/* ============================================================
 * UAC 麦克风输入（官方 usb_host_uac 组件）—— audio_in.h 的 USB 实现
 *
 * 硬件：JQ-CAM12-720P USB 三合一摄像头。音频与视频共用同一根 USB 线
 *       （UVC+UAC 复合设备），故语音输入不再需要 I2S/麦克风引脚。
 *
 * 结构：
 *   usb_lib_task   : usb_host_install + 库事件循环（组件要求先装 host）
 *   uac_setup_task : uac_host_install（后台事件任务）→ 等待 RX_CONNECTED
 *                    （麦克风）→ open → start(16k/1ch/16bit) → unmute
 *   feed 任务(voice_cmd) 经 audio_in_read → uac_host_device_read
 *
 * 历史备注：此前为手写裸 isoc 实现，遇到 IDF USBH 对该设备等时管道
 * 的提交异常（URB 全部收不到数据），改用官方组件托管全部 UAC 细节
 * （枚举/SET_INTERFACE/静音/URB 调度）。设备未接入时 read 返回
 * ESP_ERR_TIMEOUT，不伪造音频。
 * ============================================================ */

#define TAG "audio_uac"

static uac_host_device_handle_t s_mic = NULL;
static volatile bool s_streaming = false;
static volatile bool s_mic_found = false;
static uint8_t s_mic_addr = 0;
static uint8_t s_mic_iface = 0;
static SemaphoreHandle_t s_install_done = NULL;   /* usb_host_install 完成 */

/* ---------------- UAC 驱动事件：发现麦克风（RX）接口 ---------------- */
static void uac_driver_evt(uint8_t addr, uint8_t iface_num,
                           const uac_host_driver_event_t event, void *arg)
{
    (void)arg;
    if (event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
        s_mic_addr = addr;
        s_mic_iface = iface_num;
        s_mic_found = true;
        ESP_LOGI(TAG, "发现 UAC 麦克风接口：addr=%u iface=%u", addr, iface_num);
    }
}

/* ---------------- UAC 设备事件：断开/传输错误 ---------------- */
static void uac_device_evt(uac_host_device_handle_t handle,
                           const uac_host_device_event_t event, void *arg)
{
    (void)arg;
    switch (event) {
    case UAC_HOST_DRIVER_EVENT_DISCONNECTED:
        if (handle == s_mic) {
            ESP_LOGW(TAG, "麦克风设备断开（等待重新接入）");
            s_streaming = false;
            s_mic = NULL;
            s_mic_found = false;
        }
        break;
    case UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR:
        ESP_LOGW(TAG, "麦克风传输错误（继续）");
        break;
    default:
        break;
    }
}

/* ---------------- 装配流程：install → 等 RX → open → start → unmute ---------------- */
static void uac_setup_task(void *arg)
{
    (void)arg;
    xSemaphoreTake(s_install_done, portMAX_DELAY);

    const uac_host_driver_config_t drv_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_driver_evt,
        .callback_arg = NULL,
    };
    ESP_ERROR_CHECK(uac_host_install(&drv_cfg));
    ESP_LOGI(TAG, "UAC 驱动已安装：等待摄像头（USB 三合一）接入");

    for (;;) {
        if (s_mic_found && !s_streaming) {
            const uac_host_device_config_t dev_cfg = {
                .addr = s_mic_addr,
                .iface_num = s_mic_iface,
                .buffer_size = 8192,        /* ≈256ms @16k mono 16bit */
                .buffer_threshold = 2048,
                .callback = uac_device_evt,
                .callback_arg = NULL,
            };
            esp_err_t err = uac_host_device_open(&dev_cfg, &s_mic);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "device_open: %s", esp_err_to_name(err));
                s_mic_found = false;        /* 等驱动重新上报 */
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            (void)uac_host_printf_device_param(s_mic);

            const uac_host_stream_config_t stream_cfg = {
                .channels = 1,
                .bit_resolution = 16,
                .sample_freq = 16000,       /* 该麦克风原生 16k，无重采样 */
                .flags = 0,
            };
            err = uac_host_device_start(s_mic, &stream_cfg);
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "device_start(16k/1ch/16bit): %s", esp_err_to_name(err));
                uac_host_device_close(s_mic);
                s_mic = NULL;
                s_mic_found = false;
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
            /* 官方实现解除静音（此前手写 SET_CUR 已验证该设备需要） */
            err = uac_host_device_set_mute(s_mic, false);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "set_mute(false): %s（忽略，继续）", esp_err_to_name(err));
            }
            s_streaming = true;
            ESP_LOGI(TAG, "★ UAC 麦克风启动: 16000Hz 1ch 16bit");
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------- USB Host 库安装 + 库事件循环 ---------------- */
static void usb_lib_task(void *arg)
{
    (void)arg;
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_ERROR_CHECK(usb_host_install(&host_cfg));
    ESP_LOGI(TAG, "USB Host 已安装");
    xSemaphoreGive(s_install_done);
    while (1) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

/* ============================================================
 * audio_in 接口
 * ============================================================ */
esp_err_t audio_in_init(void)
{
    static bool inited = false;
    if (inited) {
        return ESP_OK;
    }
    s_install_done = xSemaphoreCreateBinary();

    if (xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL, 0) != pdPASS ||
        xTaskCreatePinnedToCore(uac_setup_task, "uac_setup", 4096, NULL, 6, NULL, 0) != pdPASS) {
        ESP_LOGE(TAG, "任务创建失败");
        return ESP_ERR_NO_MEM;
    }
    inited = true;
    ESP_LOGI(TAG, "UAC 麦克风初始化完成：摄像头接入后自动出流");
    return ESP_OK;
}

esp_err_t audio_in_read(int16_t *buf, size_t samples)
{
    if (!s_streaming || !s_mic) {
        return ESP_ERR_TIMEOUT;         /* 无流：不伪造音频，上层延时重试 */
    }
    uint32_t bytes = (uint32_t)samples * sizeof(int16_t);
    uint32_t got = 0;
    esp_err_t err = uac_host_device_read(s_mic, (uint8_t *)buf, bytes, &got,
                                         pdMS_TO_TICKS(60));
    if (err != ESP_OK) {
        return err;
    }
    if (got < bytes) {
        memset((uint8_t *)buf + got, 0, bytes - got);   /* 不足补零（AFE 容忍一帧缺字） */
    }
    return ESP_OK;
}

bool audio_in_is_streaming(void)
{
    return s_streaming;
}
