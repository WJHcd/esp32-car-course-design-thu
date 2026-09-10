#include "esp_log.h"

#include "motion.h"
#include "voice_cmd.h"
#include "app_menu.h"
#include "bt_remote.h"
#include "ram_mode.h"
#include "voice_remote.h"
#include "status_disp.h"

/* ============================================================
 * 语音功能选择（主菜单状态机）
 *
 * 见 app_menu.h 顶部状态图。模式切换统一走 app_menu_switch()：
 *   刹停 → 旧模块 exit() → 记录新模式 → 新模块 enter() → 打印可说话令。
 * 语音识别回调运行在 voice_detect 任务（core 1）上下文，
 * 本文件内只做状态切换与运动接口调用（均非阻塞），不进行耗时操作。
 * ============================================================ */

static const char *TAG = "menu";

static app_mode_t s_mode = APP_MODE_SELECT;
static bool s_inited = false;

static const char *s_mode_names[APP_MODE_MAX] = {
    "功能选择", "蓝牙遥控", "攻城槌", "语音遥控",
};

const char *app_menu_mode_name(app_mode_t mode)
{
    if (mode >= APP_MODE_MAX) {
        return "未知";
    }
    return s_mode_names[mode];
}

/* ---------- 模式切换 ---------- */
static void app_menu_switch(app_mode_t mode)
{
    if (mode == s_mode) {
        return;
    }

    /* 1. 刹停：任何模式离开都先停电机，避免旧动作残留 */
    motion_stop();

    /* 2. 退出旧模式 */
    switch (s_mode) {
    case APP_MODE_BT:
        bt_remote_exit();
        break;
    case APP_MODE_RAM:
        ram_mode_exit();
        break;
    case APP_MODE_VOICE:
        voice_remote_exit();
        break;
    default:
        break;
    }

    /* 3. 进入新模式 */
    s_mode = mode;
    switch (mode) {
    case APP_MODE_BT:
        bt_remote_enter();
        break;
    case APP_MODE_RAM:
        ram_mode_enter();
        break;
    case APP_MODE_VOICE:
        voice_remote_enter();
        break;
    default:
        break;
    }

    ESP_LOGI(TAG, "====== 当前模式：%s（说“选择功能”可返回菜单）======",
             app_menu_mode_name(s_mode));
}

/* ============================================================
 * 语音识别回调：按当前模式裁决事件
 * ============================================================ */
void app_menu_on_voice_evt(voice_evt_t evt, void *user_ctx)
{
    (void)user_ctx;
    if (evt == VOICE_EVT_NONE) {
        return;
    }

    switch (s_mode) {
    case APP_MODE_SELECT:
        /* 功能选择：听 蓝牙遥控 / 表情 / 语音遥控 */
        switch (evt) {
        case VOICE_EVT_SELECT_BT:
            app_menu_switch(APP_MODE_BT);
            break;
        case VOICE_EVT_SELECT_RAM:
            app_menu_switch(APP_MODE_RAM);
            break;
        case VOICE_EVT_SELECT_VOICE:
            app_menu_switch(APP_MODE_VOICE);
            break;
        case VOICE_EVT_BACK_SELECT:
            ESP_LOGI(TAG, "已在功能选择，请说：蓝牙遥控 / 攻城槌 / 语音遥控");
            break;
        default:
            ESP_LOGW(TAG, "功能选择模式忽略“%s”：仅支持 蓝牙遥控/攻城槌/语音遥控",
                     voice_evt_to_str(evt));
            break;
        }
        break;

    case APP_MODE_BT:
    case APP_MODE_RAM:
        /* 蓝牙遥控控制通道来自手机；攻城槌为自主流程。
         * 语音仅响应“选择功能”返回菜单 */
        if (evt == VOICE_EVT_BACK_SELECT) {
            app_menu_switch(APP_MODE_SELECT);
        } else {
            ESP_LOGW(TAG, "%s模式仅响应“选择功能”，忽略“%s”",
                     app_menu_mode_name(s_mode), voice_evt_to_str(evt));
        }
        break;

    case APP_MODE_VOICE:
        /* 语音遥控：动作命令直接执行；“选择功能”返回菜单 */
        if (evt == VOICE_EVT_BACK_SELECT) {
            app_menu_switch(APP_MODE_SELECT);
        } else {
            /* 状态屏动作回显（ASCII 文案） */
            const char *action = NULL;
            switch (evt) {
            case VOICE_EVT_DRIVE_FORWARD:  action = "FWD";   break;
            case VOICE_EVT_DRIVE_BACKWARD: action = "BWD";   break;
            case VOICE_EVT_DRIVE_LEFT:     action = "LEFT";  break;
            case VOICE_EVT_DRIVE_RIGHT:    action = "RIGHT"; break;
            case VOICE_EVT_DRIVE_STOP:     action = "STOP";  break;
            default: break;
            }
            if (action) {
                status_disp_set_action(action);
            }
            voice_remote_execute(evt);
        }
        break;

    default:
        break;
    }
}

void app_menu_init(void)
{
    if (s_inited) {
        return;
    }

    /* 电机接口 + 三个功能模块（均已实现） */
    motion_init();
    voice_remote_init();
    bt_remote_init();
    ram_mode_init();

    s_inited = true;
    s_mode = APP_MODE_SELECT;

    ESP_LOGI(TAG, "==============================================");
    ESP_LOGI(TAG, "语音功能选择就绪：免唤醒：直接说出功能名即可切换");
    ESP_LOGI(TAG, "  “蓝牙遥控”→ 蓝牙遥控   “攻城槌”→ 攻城槌   “语音遥控”→ 语音遥控");
    ESP_LOGI(TAG, "==============================================");
}

app_mode_t app_menu_get_mode(void)
{
    return s_mode;
}
