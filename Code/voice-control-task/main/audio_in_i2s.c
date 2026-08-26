#include <string.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "driver/i2s_std.h"
#include "audio_in.h"

/* ============================================================
 * I2S 数字麦克风输入（INMP441 等 MEMS 麦克风）
 *
 * ⚠️ 接线待定（先跑通逻辑）：默认占用 GPIO 4/5/6，避开 USB Host(19/20)。
 *    - BCLK  ← 麦克风 SCK/BCLK
 *    - WS    ← 麦克风 WS/LRCK
 *    - DIN   → 麦克风 DOUT/SD
 *    - INMP441 的 L/R 脚：接 GND 选左声道（配合 I2S_SLOT_MODE_MONO 任一通道均可）
 *
 * 采样率：16kHz / 16bit / 单声道（ESP-SR 输入要求）
 * ============================================================ */

#define MIC_I2S_NUM      I2S_NUM_0
#define MIC_BCLK_PIN     GPIO_NUM_5
#define MIC_WS_PIN       GPIO_NUM_4
#define MIC_DIN_PIN      GPIO_NUM_6
#define MIC_SAMPLE_RATE  16000

static const char *TAG = "audio_in";
static i2s_chan_handle_t s_rx = NULL;

esp_err_t audio_in_init(void)
{
    if (s_rx) {
        return ESP_OK;
    }

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(MIC_I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;                 /* 空读自动清零，避免底噪 */
    esp_err_t err = i2s_new_channel(&chan_cfg, NULL, &s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel: %s", esp_err_to_name(err));
        return err;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(MIC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                        I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = MIC_BCLK_PIN,
            .ws   = MIC_WS_PIN,
            .dout = I2S_GPIO_UNUSED,
            .din  = MIC_DIN_PIN,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    err = i2s_channel_init_std_mode(s_rx, &std_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode: %s", esp_err_to_name(err));
        return err;
    }

    err = i2s_channel_enable(s_rx);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2S 麦克风就绪: %d Hz / 16bit / mono (BCLK=%d WS=%d DIN=%d)",
             MIC_SAMPLE_RATE, MIC_BCLK_PIN, MIC_WS_PIN, MIC_DIN_PIN);
    return ESP_OK;
}

esp_err_t audio_in_read(int16_t *buf, size_t samples)
{
    if (!s_rx) {
        return ESP_ERR_INVALID_STATE;
    }
    size_t bytes = samples * sizeof(int16_t);
    size_t got = 0;
    esp_err_t err = i2s_channel_read(s_rx, buf, bytes, &got, portMAX_DELAY);
    if (err != ESP_OK) {
        return err;
    }
    /* 数据不足一帧时补零，避免 AFE 拿到残缺帧 */
    if (got < bytes) {
        memset((uint8_t *)buf + got, 0, bytes - got);
    }
    return ESP_OK;
}
