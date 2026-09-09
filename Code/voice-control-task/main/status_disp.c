#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "tft_disp.h"
#include "status_disp.h"
#include "app_menu.h"
#include "bt_remote.h"
#include "audio_in.h"
#include "motion.h"

/* ============================================================
 * 状态屏：布局（每行 16px，最多 9 行）
 *
 *  行0 红    标题 "VOICE CAR"
 *  行1 白    当前模式 "MODE: FUNC-SELECT / BT-REMOTE / FACE / VOICE-REMOTE"
 *  行2 黄    主提示（当前模式能说什么/做什么）
 *  行3 黄    副提示
 *  行4 绿    状态行：MIC:ON/OFF  MOTOR:ON/OFF  BLE:CONN/ARM
 *  行5 蓝    最近动作回显（>> FWD 等，4 秒后消失）
 *
 * FACE 模式（表情占屏）：进入即清屏并暂停绘制，离开自动恢复。
 * 文本全大写 ASCII（5x7 字库无中文）。
 * ============================================================ */

#define TAG "status_disp"
#define REFRESH_MS     250
#define ACTION_HOLD_MS 4000
#define ACTION_TEXT_MAX 20

static char s_action[ACTION_TEXT_MAX + 1] = "";
static TickType_t s_action_tick = 0;

void status_disp_set_action(const char *ascii_action)
{
    strncpy(s_action, ascii_action, ACTION_TEXT_MAX);
    s_action[ACTION_TEXT_MAX] = '\0';
    s_action_tick = xTaskGetTickCount();
}

/* 模式英文名/提示（与 app_menu 枚举顺序一致） */
static const char *mode_name(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "FUNC-SELECT";
    case APP_MODE_BT:     return "BT-REMOTE";
    case APP_MODE_FACE:   return "FACE";
    case APP_MODE_VOICE:  return "VOICE-REMOTE";
    default:              return "?";
    }
}

static const char *hint1(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "SAY: BT FACE VOICE";
    case APP_MODE_BT:     return "CONNECT MYCAR-BLE";
    case APP_MODE_FACE:   return "FACE MODE (TFT)";
    case APP_MODE_VOICE:  return "SAY FWD BWD L/R";
    default:              return "";
    }
}

static const char *hint2(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "WAKE-FREE LISTENING";
    case APP_MODE_BT:     return "BTN:1=ARM 2=ESTOP";
    case APP_MODE_FACE:   return "SCREEN TAKEN OVER";
    case APP_MODE_VOICE:  return "OR SAY: STOP/BACK";
    default:              return "";
    }
}

static void draw_page(void)
{
    app_mode_t mode = app_menu_get_mode();
    bool bt_conn = false, bt_armed = false;
    bt_remote_link_state(&bt_conn, &bt_armed);
    bool mic = audio_in_is_streaming();
    bool motor = motion_is_enabled();

    char line[32];

    /* 所有行定宽 20 字符覆盖绘制（5x7 字形自带黑色背景）：
     * 与 task2 同款"只重绘文本行"方式，整屏 fill 只在切页时做一次，
     * 避免每帧清屏造成的闪烁。 */
#define PAD20(l) do { size_t _n = strlen(l); if (_n > 20) { (l)[20] = '\0'; _n = 20; } \
                      while (_n < 20) (l)[_n++] = ' '; } while (0)

    /* 行0 标题 */
    tft_disp_text(0, "VOICE CAR          ", TFT_COLOR_RED);

    /* 行1 模式 */
    snprintf(line, sizeof(line), "MODE: %s", mode_name(mode));
    PAD20(line);
    tft_disp_text(1, line, TFT_COLOR_WHITE);

    /* 行2/3 提示 */
    snprintf(line, sizeof(line), "%s", hint1(mode));
    PAD20(line);
    tft_disp_text(2, line, TFT_COLOR_YELLOW);
    snprintf(line, sizeof(line), "%s", hint2(mode));
    PAD20(line);
    tft_disp_text(3, line, TFT_COLOR_YELLOW);

    /* 行4 状态 */
    snprintf(line, sizeof(line), "MIC:%s MOTOR:%s",
             mic ? "ON " : "OFF", motor ? "ON" : "OFF");
    PAD20(line);
    tft_disp_text(4, line, TFT_COLOR_GREEN);
    if (mode == APP_MODE_BT) {
        snprintf(line, sizeof(line), "BLE:%s %s",
                 bt_conn ? "CONN" : "IDLE", bt_armed ? "ARMED" : "----");
        PAD20(line);
        tft_disp_text(5, line, TFT_COLOR_GREEN);
    } else if (mode == APP_MODE_VOICE && s_action[0] &&
               xTaskGetTickCount() - s_action_tick < pdMS_TO_TICKS(ACTION_HOLD_MS)) {
        snprintf(line, sizeof(line), ">> %s", s_action);
        PAD20(line);
        tft_disp_text(5, line, TFT_COLOR_BLUE);
    } else {
        tft_disp_text(5, "                    ", TFT_COLOR_BLACK);   /* 空格覆盖清行 */
    }
#undef PAD20
}

static void disp_task(void *arg)
{
    (void)arg;
    bool face_owned = false;    /* FACE 是否占用屏幕 */
    bool page_up = false;       /* 状态页是否已铺底 */
    vTaskDelay(pdMS_TO_TICKS(600));     /* 等 app_menu/audio 初始化完成 */
    for (;;) {
        app_mode_t mode = app_menu_get_mode();
        if (mode == APP_MODE_FACE) {
            /* 表情功能占用屏幕：清屏一次并停止绘制 */
            if (!face_owned) {
                tft_disp_fill(TFT_COLOR_BLACK);
                face_owned = true;
                page_up = false;
                ESP_LOGI(TAG, "表情模式占用屏幕，状态屏让位");
            }
        } else {
            face_owned = false;
            if (!page_up) {
                tft_disp_fill(TFT_COLOR_BLACK);     /* 仅切回状态页时清一次 */
                page_up = true;
            }
            draw_page();
        }
        vTaskDelay(pdMS_TO_TICKS(REFRESH_MS));
    }
}

void status_disp_init(void)
{
    static bool inited = false;
    if (inited) {
        return;
    }
    tft_disp_init();
    if (xTaskCreatePinnedToCore(disp_task, "status_disp", 3072, NULL, 2, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "状态屏任务创建失败");
        return;
    }
    inited = true;
    ESP_LOGI(TAG, "状态屏已启动（模式/BLE/电机/麦克风/动作回显；FACE 占屏让位）");
}
