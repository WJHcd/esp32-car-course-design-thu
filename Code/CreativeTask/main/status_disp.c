#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "display.h"
#include "status_disp.h"
#include "app_menu.h"
#include "bt_remote.h"
#include "usb_audio.h"
#include "motion.h"

/* ============================================================
 * 状态屏：功能选择/蓝牙/语音遥控模式下显示模式信息（移植自
 * voice-control-task 的 status_disp，绘制后端换成 QR 工程的
 * display.c 帧缓冲驱动——与 GIF 动画共用同一驱动，避免双驱动抢屏）。
 *
 * 布局（每行 16px，9 行）：
 *   行0 红  标题 "RAM CAR"
 *   行1 白  当前模式 MODE: FUNC-SELECT / BT-REMOTE / RAM / VOICE-REMOTE
 *   行2/3 黄 提示（当前模式能说什么/做什么）
 *   行4 绿  状态 MIC:ON/OFF MOTOR:ON/OFF
 *   行5 蓝  BLE 状态 / 语音动作回显（4 秒后消失）
 *
 * 攻城槌模式占用屏幕（GIF 动画）：清屏一次并暂停绘制，离开自动恢复。
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

/* 模式英文名/提示 */
static const char *mode_name(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "FUNC-SELECT";
    case APP_MODE_BT:     return "BT-REMOTE";
    case APP_MODE_RAM:    return "BATTERING-RAM";
    case APP_MODE_VOICE:  return "VOICE-REMOTE";
    default:              return "?";
    }
}

static const char *hint1(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "SAY: BT RAM VOICE";
    case APP_MODE_BT:     return "CONNECT MYCAR-BLE";
    case APP_MODE_RAM:    return "QR-1=TARGET BOX";
    case APP_MODE_VOICE:  return "SAY FWD BWD L/R";
    default:              return "";
    }
}

static const char *hint2(app_mode_t mode)
{
    switch (mode) {
    case APP_MODE_SELECT: return "WAKE-FREE LISTENING";
    case APP_MODE_BT:     return "BTN:1=ARM 2=ESTOP";
    case APP_MODE_RAM:    return "LOOP: HIT+BACKOFF";
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

/* 所有行定宽 20 字符覆盖绘制（字形自带黑底），避免每帧清屏闪烁 */
#define PAD20(l) do { size_t _n = strlen(l); if (_n > 20) { (l)[20] = '\0'; _n = 20; } \
                      while (_n < 20) (l)[_n++] = ' '; } while (0)

    /* 行0 标题 */
    display_text(0, "RAM CAR            ", 0xF800);

    /* 行1 模式 */
    snprintf(line, sizeof(line), "MODE: %s", mode_name(mode));
    PAD20(line);
    display_text(1, line, 0xFFFF);

    /* 行2/3 提示 */
    snprintf(line, sizeof(line), "%s", hint1(mode));
    PAD20(line);
    display_text(2, line, 0xFFE0);
    snprintf(line, sizeof(line), "%s", hint2(mode));
    PAD20(line);
    display_text(3, line, 0xFFE0);

    /* 行4 状态 */
    snprintf(line, sizeof(line), "MIC:%s MOTOR:%s",
             mic ? "ON " : "OFF", motor ? "ON" : "OFF");
    PAD20(line);
    display_text(4, line, 0x07E0);
    if (mode == APP_MODE_BT) {
        snprintf(line, sizeof(line), "BLE:%s %s",
                 bt_conn ? "CONN" : "IDLE", bt_armed ? "ARMED" : "----");
        PAD20(line);
        display_text(5, line, 0x07E0);
    } else if (mode == APP_MODE_VOICE && s_action[0] &&
               xTaskGetTickCount() - s_action_tick < pdMS_TO_TICKS(ACTION_HOLD_MS)) {
        snprintf(line, sizeof(line), ">> %s", s_action);
        PAD20(line);
        display_text(5, line, 0x001F);
    } else {
        display_text(5, "                    ", 0x0000);   /* 空格覆盖清行 */
    }
#undef PAD20
}

static void disp_task(void *arg)
{
    (void)arg;
    bool ram_owned = false;     /* 攻城槌是否占用屏幕 */
    bool page_up = false;       /* 状态页是否已铺底 */
    vTaskDelay(pdMS_TO_TICKS(600));     /* 等 app_menu/audio 初始化完成 */
    for (;;) {
        app_mode_t mode = app_menu_get_mode();
        if (mode == APP_MODE_RAM) {
            /* 攻城槌 GIF 动画占用屏幕：停止绘制 */
            if (!ram_owned) {
                ram_owned = true;
                page_up = false;
                ESP_LOGI(TAG, "攻城槌模式占用屏幕，状态屏让位");
            }
        } else {
            ram_owned = false;
            if (!page_up) {
                display_fill(0x0000);     /* 仅切回状态页时清一次 */
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
    display_init();
    if (xTaskCreatePinnedToCore(disp_task, "status_disp", 3072, NULL, 2, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "状态屏任务创建失败");
        return;
    }
    inited = true;
    ESP_LOGI(TAG, "状态屏已启动（模式/BLE/电机/麦克风/动作回显；攻城槌占屏让位）");
}
