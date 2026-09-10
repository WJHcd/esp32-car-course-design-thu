#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "usb/usb_host.h"
#include "usb/uac_host.h"

#include "usb_audio.h"

/* ============================================================
 * UAC 麦克风（RX）+ 喇叭（TX）—— JQ-CAM12 USB 三合一
 *
 * 麦克风：沿用 voice-control-task audio_in_uac.c 的已验证流程
 *   （官方 esp_uac_host 组件托管枚举/SET_INTERFACE/静音/URB）。
 * 喇叭：同一设备的 TX 接口。流参数自动协商：依次尝试
 *   16k/1ch → 16k/2ch → 48k/1ch → 48k/2ch，选中即写 PCM；
 *   嵌入的 WAV 为 16k 与 48k 两套单声道版本，双声道由软件升采样。
 *
 * 一个 uac_host 驱动只有一个 driver 回调，RX/TX 事件在此统一分发。
 * usb_host_install 由 main 的 camera_task_start() 先行完成；
 * 若未装（独立调试本模块），这里补装并起库事件任务。
 * ============================================================ */

#define TAG "uac"

/* ---------------- 嵌入的 WAV 资源（CMake EMBED_FILES） ---------------- */
extern const uint8_t snd_search_16k_start[] asm("_binary_snd_search_16k_wav_start");
extern const uint8_t snd_search_16k_end[]   asm("_binary_snd_search_16k_wav_end");
extern const uint8_t snd_attack_16k_start[] asm("_binary_snd_attack_16k_wav_start");
extern const uint8_t snd_attack_16k_end[]   asm("_binary_snd_attack_16k_wav_end");
extern const uint8_t snd_search_48k_start[] asm("_binary_snd_search_48k_wav_start");
extern const uint8_t snd_search_48k_end[]   asm("_binary_snd_search_48k_wav_end");
extern const uint8_t snd_attack_48k_start[] asm("_binary_snd_attack_48k_wav_start");
extern const uint8_t snd_attack_48k_end[]   asm("_binary_snd_attack_48k_wav_end");

typedef struct {
    const uint8_t *start;
    const uint8_t *end;
} wav_blob_t;

/* [rate_idx: 0=16k 1=48k][snd] */
static const wav_blob_t s_wav[2][AUDIO_SND_MAX] = {
    [0] = { [AUDIO_SND_SEARCH] = { snd_search_16k_start, snd_search_16k_end },
            [AUDIO_SND_ATTACK] = { snd_attack_16k_start, snd_attack_16k_end } },
    [1] = { [AUDIO_SND_SEARCH] = { snd_search_48k_start, snd_search_48k_end },
            [AUDIO_SND_ATTACK] = { snd_attack_48k_start, snd_attack_48k_end } },
};

/* ---------------- UAC 状态 ---------------- */
static uac_host_device_handle_t s_mic = NULL;
static volatile bool s_mic_streaming = false;
static volatile bool s_mic_found = false;
static uint8_t s_mic_addr, s_mic_iface;

static uac_host_device_handle_t s_spk = NULL;
static volatile bool s_spk_ready = false;     /* 已 open+start */
static volatile bool s_spk_found = false;
static uint8_t s_spk_addr, s_spk_iface;
static uint32_t s_spk_rate = 0;               /* 协商成功的采样率 */
static uint8_t  s_spk_ch = 0;                 /* 协商成功的声道数 */

static TaskHandle_t s_play_task_handle = NULL;

/* ---------------- WAV 解析：跳过头部，返回 PCM 数据区 ---------------- */
static bool wav_payload(const wav_blob_t *w, const uint8_t **pcm, size_t *len)
{
    const uint8_t *p = w->start;
    size_t n = (size_t)(w->end - w->start);
    if (n < 44 || memcmp(p, "RIFF", 4) != 0) {
        return false;
    }
    for (size_t off = 12; off + 8 <= n; ) {
        uint32_t chunk_len = (uint32_t)p[off + 4] | ((uint32_t)p[off + 5] << 8) |
                             ((uint32_t)p[off + 6] << 16) | ((uint32_t)p[off + 7] << 24);
        if (memcmp(p + off, "data", 4) == 0) {
            size_t avail = n - (off + 8);
            *pcm = p + off + 8;
            *len = chunk_len < avail ? chunk_len : avail;
            return true;
        }
        off += 8 + chunk_len + (chunk_len & 1);
    }
    return false;
}

/* ---------------- 驱动事件：发现 RX/TX 接口 ---------------- */
static void uac_driver_evt(uint8_t addr, uint8_t iface_num,
                           const uac_host_driver_event_t event, void *arg)
{
    (void)arg;
    if (event == UAC_HOST_DRIVER_EVENT_RX_CONNECTED) {
        s_mic_addr = addr;
        s_mic_iface = iface_num;
        s_mic_found = true;
        ESP_LOGI(TAG, "发现 UAC 麦克风接口：addr=%u iface=%u", addr, iface_num);
    } else if (event == UAC_HOST_DRIVER_EVENT_TX_CONNECTED) {
        s_spk_addr = addr;
        s_spk_iface = iface_num;
        s_spk_found = true;
        ESP_LOGI(TAG, "发现 UAC 喇叭接口：addr=%u iface=%u", addr, iface_num);
    }
}

static void uac_device_evt(uac_host_device_handle_t handle,
                           const uac_host_device_event_t event, void *arg)
{
    (void)arg;
    if (event == UAC_HOST_DRIVER_EVENT_DISCONNECTED) {
        if (handle == s_mic) {
            ESP_LOGW(TAG, "麦克风断开（等待重新接入）");
            s_mic_streaming = false;
            s_mic = NULL;
            s_mic_found = false;
        } else if (handle == s_spk) {
            ESP_LOGW(TAG, "喇叭断开");
            s_spk_ready = false;
            s_spk = NULL;
            s_spk_found = false;
        }
    } else if (event == UAC_HOST_DEVICE_EVENT_TRANSFER_ERROR) {
        ESP_LOGW(TAG, "UAC 传输错误（继续）");
    }
}

/* 喇叭流参数候选：16k 优先（与嵌入 WAV 对应），失败退 48k */
static const uac_host_stream_config_t s_spk_candidates[] = {
    { .channels = 1, .bit_resolution = 16, .sample_freq = 16000, .flags = 0 },
    { .channels = 2, .bit_resolution = 16, .sample_freq = 16000, .flags = 0 },
    { .channels = 1, .bit_resolution = 16, .sample_freq = 48000, .flags = 0 },
    { .channels = 2, .bit_resolution = 16, .sample_freq = 48000, .flags = 0 },
};

static esp_err_t spk_open_and_start(void)
{
    const uac_host_device_config_t dev_cfg = {
        .addr = s_spk_addr,
        .iface_num = s_spk_iface,
        .buffer_size = 16384,
        .buffer_threshold = 4096,
        .callback = uac_device_evt,
        .callback_arg = NULL,
    };
    esp_err_t err = uac_host_device_open(&dev_cfg, &s_spk);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "喇叭 device_open: %s", esp_err_to_name(err));
        s_spk_found = false;
        return err;
    }
    for (size_t i = 0; i < sizeof(s_spk_candidates) / sizeof(s_spk_candidates[0]); i++) {
        err = uac_host_device_start(s_spk, &s_spk_candidates[i]);
        if (err == ESP_OK) {
            s_spk_rate = s_spk_candidates[i].sample_freq;
            s_spk_ch = s_spk_candidates[i].channels;
            break;
        }
        ESP_LOGW(TAG, "喇叭 %lukHz/%dch 协商失败: %s",
                 (unsigned long)s_spk_candidates[i].sample_freq,
                 s_spk_candidates[i].channels, esp_err_to_name(err));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "喇叭所有流参数均失败");
        uac_host_device_close(s_spk);
        s_spk = NULL;
        s_spk_found = false;
        return err;
    }
    if (uac_host_device_set_mute(s_spk, false) != ESP_OK) {
        ESP_LOGW(TAG, "喇叭解除静音失败（忽略）");
    }
    s_spk_ready = true;
    ESP_LOGI(TAG, "★ UAC 喇叭启动: %luHz %dch 16bit", (unsigned long)s_spk_rate, s_spk_ch);
    return ESP_OK;
}

static void mic_open_and_start(void)
{
    const uac_host_device_config_t dev_cfg = {
        .addr = s_mic_addr,
        .iface_num = s_mic_iface,
        .buffer_size = 8192,
        .buffer_threshold = 2048,
        .callback = uac_device_evt,
        .callback_arg = NULL,
    };
    if (uac_host_device_open(&dev_cfg, &s_mic) != ESP_OK) {
        ESP_LOGE(TAG, "麦克风 device_open 失败，等驱动重新上报");
        s_mic_found = false;
        return;
    }
    (void)uac_host_printf_device_param(s_mic);
    const uac_host_stream_config_t stream_cfg = {
        .channels = 1,
        .bit_resolution = 16,
        .sample_freq = 16000,   /* 该麦克风原生 16k，无重采样 */
        .flags = 0,
    };
    if (uac_host_device_start(s_mic, &stream_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "麦克风 device_start(16k/1ch) 失败");
        uac_host_device_close(s_mic);
        s_mic = NULL;
        s_mic_found = false;
        return;
    }
    if (uac_host_device_set_mute(s_mic, false) != ESP_OK) {
        ESP_LOGW(TAG, "麦克风解除静音失败（忽略）");
    }
    s_mic_streaming = true;
    ESP_LOGI(TAG, "★ UAC 麦克风启动: 16000Hz 1ch 16bit");
}

/* ---------------- 装配轮询任务：mic 出流 / spk 按需开流 ---------------- */
static void uac_setup_task(void *arg)
{
    (void)arg;
    for (;;) {
        if (s_mic_found && !s_mic_streaming) {
            mic_open_and_start();
        }
        if (s_spk_found && !s_spk_ready) {
            spk_open_and_start();
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* ---------------- USB Host 安装（若主流程尚未装） ---------------- */
static void usb_lib_task(void *arg)
{
    (void)arg;
    while (1) {
        uint32_t flags = 0;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
    }
}

static esp_err_t ensure_usb_host(void)
{
    esp_err_t err = usb_host_install(&(const usb_host_config_t) {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    });
    if (err == ESP_ERR_INVALID_STATE) {
        return ESP_OK;      /* main 已装过 */
    }
    if (err != ESP_OK) {
        return err;
    }
    return xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL, 0)
               == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

/* ---------------- 播放任务：循环写选中的 WAV ---------------- */
static volatile audio_snd_t s_req_snd = AUDIO_SND_SEARCH;
static volatile bool s_play_req = false;
static volatile bool s_playing = false;

static void audio_play_task(void *arg)
{
    (void)arg;
    static int16_t out_buf[1024];   /* 一次写 512 个单声道样本（含升声道） */
    for (;;) {
        if (!s_play_req) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        s_play_req = false;
        s_playing = true;

        const int rate_idx = (s_spk_rate >= 48000) ? 1 : 0;
        const uint8_t *pcm = NULL;
        size_t pcm_len = 0;
        if (!s_spk_ready ||
            !wav_payload(&s_wav[rate_idx][s_req_snd], &pcm, &pcm_len)) {
            ESP_LOGE(TAG, "喇叭未就绪或 WAV 无效，放弃播放");
            s_playing = false;
            continue;
        }
        ESP_LOGI(TAG, "▶ 播放音效 %d（%luHz %dch，%uKB，循环）", s_req_snd,
                 (unsigned long)s_spk_rate, s_spk_ch, (unsigned)(pcm_len / 1024));

        const int16_t *p = (const int16_t *)pcm;
        const size_t total = pcm_len / sizeof(int16_t);
        size_t pos = 0;
        while (s_playing && s_spk_ready && !s_play_req) {
            /* 取一段单声道样本，必要时升为双声道 */
            size_t nsamp = sizeof(out_buf) / sizeof(int16_t) / (s_spk_ch == 2 ? 2 : 1);
            if (pos + nsamp > total) {
                nsamp = total - pos;    /* 到尾：写完剩余再从头循环 */
            }
            size_t out_n = 0;
            for (size_t k = 0; k < nsamp; k++) {
                out_buf[out_n++] = p[pos + k];
                if (s_spk_ch == 2) {
                    out_buf[out_n++] = p[pos + k];
                }
            }
            uint32_t written = 0;
            esp_err_t err = uac_host_device_write(s_spk, (uint8_t *)out_buf,
                                                  out_n * sizeof(int16_t),
                                                  200);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "喇叭 write: %s", esp_err_to_name(err));
                vTaskDelay(pdMS_TO_TICKS(50));
                continue;
            }
            pos += nsamp;   /* write 成功即整段写入 */
            if (pos >= total) {
                pos = 0;    /* 循环 */
            }
        }
        s_playing = false;
    }
}

/* ============================================================
 * 对外接口
 * ============================================================ */
esp_err_t audio_in_init(void)
{
    static bool inited = false;
    if (inited) {
        return ESP_OK;
    }

    esp_err_t err = ensure_usb_host();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(err));
        return err;
    }

    const uac_host_driver_config_t drv_cfg = {
        .create_background_task = true,
        .task_priority = 5,
        .stack_size = 4096,
        .core_id = 0,
        .callback = uac_driver_evt,
        .callback_arg = NULL,
    };
    err = uac_host_install(&drv_cfg);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "uac_host_install: %s", esp_err_to_name(err));
        return err;
    }

    if (xTaskCreatePinnedToCore(uac_setup_task, "uac_setup", 4096, NULL, 6, NULL, 0) != pdPASS ||
        xTaskCreatePinnedToCore(audio_play_task, "audio_play", 4096, NULL, 4, NULL, 1) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }
    inited = true;
    ESP_LOGI(TAG, "UAC 音频初始化完成（麦克风自动出流；喇叭首次播放时按需启动）");
    return ESP_OK;
}

esp_err_t audio_in_read(int16_t *buf, size_t samples)
{
    if (!s_mic_streaming || !s_mic) {
        return ESP_ERR_TIMEOUT;
    }
    uint32_t bytes = (uint32_t)samples * sizeof(int16_t);
    uint32_t got = 0;
    esp_err_t err = uac_host_device_read(s_mic, (uint8_t *)buf, bytes, &got,
                                         pdMS_TO_TICKS(60));
    if (err != ESP_OK) {
        return err;
    }
    if (got < bytes) {
        memset((uint8_t *)buf + got, 0, bytes - got);
    }
    return ESP_OK;
}

bool audio_in_is_streaming(void)
{
    return s_mic_streaming;
}

esp_err_t audio_out_play(audio_snd_t snd)
{
    if (snd >= AUDIO_SND_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_playing || s_req_snd != snd) {
        s_req_snd = snd;
        s_play_req = true;
    }
    return ESP_OK;
}

void audio_out_stop(void)
{
    s_playing = false;
    s_play_req = false;
}

bool audio_out_is_playing(void)
{
    return s_playing;
}
