#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

/* ESP-SR 语音识别（离线，v2.x API） */
#include "esp_wn_iface.h"
#include "esp_wn_models.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "model_path.h"

#include "audio_in.h"
#include "voice_cmd.h"

/* ============================================================
 * 语音识别核心（ESP-SR）
 *
 * 参考官方示例：esp-skainet/examples/cn_speech_commands_recognition
 * 模型从 flash 的 "model" 分区加载（见 partitions.csv），
 * 唤醒词模型通过 menuconfig 选择（如 Hi,乐鑫 wn9_hilexin）。
 * ============================================================ */

static const char *TAG = "voice";

/* ---------- 命令词表：顺序与枚举 ID 一一对应 ----------
 * 中文命令词由 MultiNet（中文模型）离线识别，无需训练。
 * 注意：命令词修改后需重新烧录（模型不变，仅命令表变化）。
 */
static const char *s_phrases[VOICE_CMD_MAX] = {
    [VOICE_CMD_NONE]    = NULL,
    [VOICE_CMD_FORWARD]  = "前进",
    [VOICE_CMD_BACKWARD] = "倒退",
    [VOICE_CMD_LEFT]     = "左转",
    [VOICE_CMD_RIGHT]    = "右转",
    [VOICE_CMD_UTURN]    = "调头",
    [VOICE_CMD_STOP]     = "停止",
};

static voice_cmd_cb_t s_cb = NULL;
static void *s_cb_ctx = NULL;

static srmodel_list_t *s_models = NULL;
static const esp_afe_sr_iface_t *s_afe = NULL;
static esp_afe_sr_data_t *s_afe_data = NULL;
static esp_mn_iface_t *s_mn = NULL;
static model_iface_data_t *s_mn_data = NULL;
static volatile bool s_running = false;

/* ============================================================
 * feed 任务：麦克风 → AFE（仅搬数据，放 core 1）
 * ============================================================ */
static void feed_task(void *arg)
{
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int chunk = s_afe->get_feed_chunksize(afe_data);
    int16_t *buf = malloc(chunk * sizeof(int16_t));
    if (!buf) {
        ESP_LOGE(TAG, "feed buf alloc failed (%d bytes)", chunk * 2);
        vTaskDelete(NULL);
        return;
    }

    while (s_running) {
        if (audio_in_read(buf, chunk) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        s_afe->feed(afe_data, buf);
    }

    free(buf);
    vTaskDelete(NULL);
}

/* ============================================================
 * detect 任务：AFE → 唤醒 → 命令词 → 回调（放 core 1）
 * ============================================================ */
static void detect_task(void *arg)
{
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int afe_chunk = s_afe->get_fetch_chunksize(afe_data);
    bool wakeup = false;

    ESP_LOGI(TAG, "------------ 待唤醒（说“嗨 乐鑫”） ------------");

    while (s_running) {
        afe_fetch_result_t *res = s_afe->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL) {
            ESP_LOGE(TAG, "AFE fetch 错误，语音识别退出");
            break;
        }

        /* 唤醒检测 */
        if (res->wakeup_state == WAKENET_DETECTED) {
            ESP_LOGI(TAG, "★ 已唤醒，请说命令（前进/倒退/左转/右转/调头/停止）");
            s_mn->clean(s_mn_data);
            wakeup = true;
        }

        if (!wakeup) {
            continue;
        }

        /* 命令词识别 */
        esp_mn_state_t st = s_mn->detect(s_mn_data, res->data);
        if (st == ESP_MN_STATE_DETECTING) {
            continue;
        }

        if (st == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *r = s_mn->get_results(s_mn_data);
            voice_cmd_t cmd = VOICE_CMD_NONE;
            for (int i = 0; i < r->num; i++) {
                ESP_LOGI(TAG, "TOP%d: command_id=%d phrase_id=%d string=%s prob=%.3f",
                         i + 1, r->command_id[i], r->phrase_id[i], r->string, r->prob[i]);
                /* 取置信度最高的有效命令（command_id 即枚举值 1..6） */
                if (cmd == VOICE_CMD_NONE &&
                    r->command_id[i] > VOICE_CMD_NONE &&
                    r->command_id[i] < VOICE_CMD_MAX) {
                    cmd = (voice_cmd_t)r->command_id[i];
                }
            }
            if (cmd != VOICE_CMD_NONE && s_cb) {
                ESP_LOGI(TAG, "→ 执行命令: %s", voice_cmd_to_str(cmd));
                s_cb(cmd, s_cb_ctx);
            }
        }

        if (st == ESP_MN_STATE_TIMEOUT || st == ESP_MN_STATE_DETECTED) {
            /* 超时或已识别：回到待唤醒状态 */
            s_afe->enable_wakenet(afe_data);
            wakeup = false;
            ESP_LOGI(TAG, "------------ 待唤醒 ------------");
        }
    }

    vTaskDelete(NULL);
}

const char *voice_cmd_to_str(voice_cmd_t cmd)
{
    if (cmd <= VOICE_CMD_NONE || cmd >= VOICE_CMD_MAX) {
        return "未知";
    }
    return s_phrases[cmd];
}

esp_err_t voice_cmd_init(voice_cmd_cb_t cb, void *user_ctx)
{
    s_cb = cb;
    s_cb_ctx = user_ctx;

    /* 1. 音频输入（I2S 麦克风） */
    esp_err_t err = audio_in_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "音频输入初始化失败: %s", esp_err_to_name(err));
        return err;
    }

    /* 2. 加载 flash "model" 分区中的模型 */
    s_models = esp_srmodel_init("model");
    if (!s_models) {
        ESP_LOGE(TAG, "模型加载失败：请检查分区表包含 model 分区且已烧录模型");
        return ESP_ERR_NOT_FOUND;
    }
    char *wn_name = esp_srmodel_filter(s_models, ESP_WN_PREFIX, NULL);          /* 唤醒词 */
    char *mn_name = esp_srmodel_filter(s_models, ESP_MN_PREFIX, ESP_MN_CHINESE); /* 中文命令词 */
    if (!wn_name || !mn_name) {
        ESP_LOGE(TAG, "未找到唤醒词/中文命令词模型：请在 menuconfig 中选择唤醒词 "
                 "(Component config → ESP-SR)");
        return ESP_ERR_NOT_FOUND;
    }
    ESP_LOGI(TAG, "模型: 唤醒词=%s 命令词=%s", wn_name, mn_name);

    /* 3. AFE 声学前端（单麦克风，低功耗模式）
     * input_format: "M"=单麦克风，"MR"=麦克风+播放参考，"MMNR"=多路麦克风
     */
    afe_config_t *afe_cfg = afe_config_init("M", s_models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!afe_cfg) {
        ESP_LOGE(TAG, "afe_config_init 失败");
        return ESP_ERR_NO_MEM;
    }
    afe_cfg->wakenet_model_name = wn_name;
    s_afe = esp_afe_handle_from_config(afe_cfg);
    s_afe_data = s_afe->create_from_config(afe_cfg);
    afe_config_free(afe_cfg);
    if (!s_afe_data) {
        ESP_LOGE(TAG, "AFE create 失败");
        return ESP_ERR_NO_MEM;
    }

    /* 4. MultiNet 命令词识别 + 注册命令（无需训练模型） */
    s_mn = esp_mn_handle_from_name(mn_name);
    s_mn_data = s_mn->create(mn_name, 6000);
    if (!s_mn_data) {
        ESP_LOGE(TAG, "MultiNet create 失败");
        return ESP_ERR_NO_MEM;
    }
    esp_mn_commands_alloc(s_mn, s_mn_data);      /* 初始化命令词链表 */
    esp_mn_commands_clear();
    for (int id = 1; id < VOICE_CMD_MAX; id++) {
        esp_mn_commands_add(id, s_phrases[id]);   /* id 即 voice_cmd_t 枚举值 */
    }
    esp_mn_commands_update();
    s_mn->print_active_speech_commands(s_mn_data);

    /* 5. 启动 feed / detect 任务（core 1，与摄像头图像处理 core 0 分离） */
    s_running = true;
    if (xTaskCreatePinnedToCore(feed_task, "voice_feed", 8192, s_afe_data, 5, NULL, 1) != pdPASS ||
        xTaskCreatePinnedToCore(detect_task, "voice_detect", 8192, s_afe_data, 5, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "任务创建失败");
        s_running = false;
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG, "语音识别已启动：说“嗨 乐鑫”唤醒，然后下达命令");
    return ESP_OK;
}
