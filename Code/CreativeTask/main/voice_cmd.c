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
 * 唤醒词模型通过 menuconfig 选择（如 Hi,乐鑫 wn9_hilexin）；免唤醒
 * 模式下模型保留加载但已 disable，仅命令词模型全程监听。
 *
 * 识别流程（免唤醒连续识别）：
 *   AFE 创建后 disable_wakenet → MultiNet 逐帧监听（自带 6s 窗口）
 *          → 命中命令 → 回调 app_menu（按当前模式裁决）
 *          → 清空窗口继续监听
 * ============================================================ */

static const char *TAG = "voice";

/* ---------- 命令词表 ----------
 * 每个词条占用一个命令 ID（注册顺序即 ID：1..N），词条可共享事件
 * （如 后退/倒退 都映射 VOICE_EVT_DRIVE_BACKWARD）。
 *
 * ⚠️ MultiNet 命令短语必须是**拼音**（小写、空格分词），中文会报
 *    "invalid, please check format"（esp-sr v2 格式要求，2026-09-09
 *    实机验证）；display 为中文名仅用于日志/提示。
 * ⚠️ 新增命令词只需在词表加一行 + 烧录；是否在当前模式生效由
 *    app_menu 裁决，词表本身不分模式。
 */
typedef struct {
    voice_evt_t evt;        /* 识别后上报的事件 */
    const char *mn_phrase;  /* MultiNet 拼音短语（识别用） */
    const char *display;    /* 中文名（日志/提示用） */
} voice_phrase_t;

static const voice_phrase_t s_phrases[] = {
    /* 功能选择菜单 */
    { VOICE_EVT_SELECT_BT,    "lan ya yao kong",  "蓝牙遥控" },
    { VOICE_EVT_SELECT_RAM,   "gong cheng chui",  "攻城槌" },
    { VOICE_EVT_SELECT_VOICE, "yu yin yao kong",  "语音遥控" },
    /* 语音遥控动作 */
    { VOICE_EVT_DRIVE_FORWARD,  "qian jin",       "前进" },
    { VOICE_EVT_DRIVE_BACKWARD, "hou tui",        "后退" },
    { VOICE_EVT_DRIVE_BACKWARD, "dao tui",        "倒退" },   /* 旧版叫法，同义保留 */
    { VOICE_EVT_DRIVE_LEFT,     "zuo zhuan",      "左转" },
    { VOICE_EVT_DRIVE_RIGHT,    "you zhuan",      "右转" },
    { VOICE_EVT_DRIVE_STOP,     "ting zhi",       "停止" },
    /* 通用 */
    { VOICE_EVT_BACK_SELECT,  "xuan ze gong neng", "选择功能" },
};
#define PHRASE_COUNT (sizeof(s_phrases) / sizeof(s_phrases[0]))

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
 * detect 任务：AFE → MultiNet 连续识别 → 回调（放 core 1）
 *
 * 【免唤醒模式】不依赖 WakeNet：AFE 创建后立即 disable_wakenet，
 * 每一帧都喂 MultiNet 识别（MN 自带 6s 窗口，命中/超时后自动重开），
 * 直接说命令词即可执行。wakeup_state 字段全程忽略。
 *
 * 误触发控制：命令是否生效由 app_menu 按当前模式裁决
 * （功能选择模式只有 蓝牙遥控/攻城槌/语音遥控 会动作）。
 * ============================================================ */
static void detect_task(void *arg)
{
    esp_afe_sr_data_t *afe_data = (esp_afe_sr_data_t *)arg;
    int fetch_err = 0;      /* 连续无数据计数（节流告警用） */
    int chunk = s_afe->get_fetch_chunksize(afe_data);   /* 单帧样本数 */
    uint32_t lvl_frames = 0;
    int lvl_peak = 0;
    uint32_t mn_timeouts = 0;

    ESP_LOGI(TAG, "---- 聆听中（免唤醒）：直接说：蓝牙遥控/攻城槌/语音遥控 ----");

    while (s_running) {
        afe_fetch_result_t *res = s_afe->fetch(afe_data);
        if (!res) {
            /* 暂无完整帧，慢速轮询等待 */
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        if (res->ret_value == ESP_FAIL) {
            /* 麦克风未出流/偶发无数据时 AFE 会持续返回失败：
             * 只告警节流，不退出——音频恢复后识别自动继续 */
            if (++fetch_err == 1 || fetch_err % 200 == 0) {
                ESP_LOGW(TAG, "AFE 无数据（麦克风未出流?）已等 %d 次", fetch_err);
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        fetch_err = 0;

        /* 电平表（诊断）：每约 2s 打一次该时段的峰值幅度。
         * 正常说话峰值应到 2000~30000；恒为 0=麦克风静音/坏流；
         * 常年 <300=音量过低，需在 audio_in_uac.c 调 UAC_INPUT_GAIN。 */
        if (chunk > 0 && chunk < 4096) {
            const int16_t *p = res->data;
            for (int i = 0; i < chunk; i++) {
                int v = p[i] >= 0 ? p[i] : -p[i];
                if (v > lvl_peak) lvl_peak = v;
            }
            if (++lvl_frames >= 125) {
                ESP_LOGI(TAG, "MIC LEVEL 峰值=%d（说话应 2000~30000，静音/无声≈0~50）",
                         lvl_peak);
                lvl_frames = 0;
                lvl_peak = 0;
            }
        }

        /* 免唤醒：忽略唤醒状态位，直接连续命令识别 */
        esp_mn_state_t st = s_mn->detect(s_mn_data, res->data);
        if (st == ESP_MN_STATE_DETECTING) {
            continue;
        }

        if (st == ESP_MN_STATE_DETECTED) {
            esp_mn_results_t *r = s_mn->get_results(s_mn_data);
            voice_evt_t evt = VOICE_EVT_NONE;
            for (int i = 0; i < r->num; i++) {
                ESP_LOGI(TAG, "TOP%d: command_id=%d phrase_id=%d string=%s prob=%.3f",
                         i + 1, r->command_id[i], r->phrase_id[i], r->string, r->prob[i]);
                /* 取置信度最高的词条（command_id = 词表序号 1..N） */
                if (evt == VOICE_EVT_NONE &&
                    r->command_id[i] >= 1 && r->command_id[i] <= (int)PHRASE_COUNT) {
                    evt = s_phrases[r->command_id[i] - 1].evt;
                }
            }
            if (evt != VOICE_EVT_NONE && s_cb) {
                ESP_LOGI(TAG, "→ 识别事件: %s", voice_evt_to_str(evt));
                s_cb(evt, s_cb_ctx);
            }
        }

        /* 命中或超时：清空窗口，立即开始新一轮监听 */
        if (st == ESP_MN_STATE_TIMEOUT || st == ESP_MN_STATE_DETECTED) {
            if (st == ESP_MN_STATE_TIMEOUT && ++mn_timeouts % 5 == 0) {
                ESP_LOGI(TAG, "识别窗口超时 %lu 次（管线存活；说话未命中命令）",
                         (unsigned long)mn_timeouts);
            }
            s_mn->clean(s_mn_data);
        }
    }

    vTaskDelete(NULL);
}

const char *voice_evt_to_str(voice_evt_t evt)
{
    if (evt == VOICE_EVT_NONE) {
        return "无";
    }
    for (int i = 0; i < (int)PHRASE_COUNT; i++) {
        if (s_phrases[i].evt == evt) {
            return s_phrases[i].display;
        }
    }
    return "未知";
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
    /* 【免唤醒】唤醒词模型保留加载（保证 AFE 管线完整），
     * 但创建后立即关闭——识别不依赖唤醒，直接连续听命令 */
    if (s_afe->disable_wakenet) {
        s_afe->disable_wakenet(s_afe_data);
    }

    /* 4. MultiNet 命令词识别 + 注册命令词表（无需训练模型） */
    s_mn = esp_mn_handle_from_name(mn_name);
    s_mn_data = s_mn->create(mn_name, 6000);
    if (!s_mn_data) {
        ESP_LOGE(TAG, "MultiNet create 失败");
        return ESP_ERR_NO_MEM;
    }
    esp_mn_commands_alloc(s_mn, s_mn_data);      /* 初始化命令词链表 */
    esp_mn_commands_clear();
    for (int i = 0; i < (int)PHRASE_COUNT; i++) {
        esp_mn_commands_add(i + 1, s_phrases[i].mn_phrase);   /* 命令 ID = 词表序号 */
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

    ESP_LOGI(TAG, "语音识别已启动（免唤醒）：直接说 蓝牙遥控/攻城槌/语音遥控");
    return ESP_OK;
}
