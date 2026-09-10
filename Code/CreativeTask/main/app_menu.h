#pragma once
#include "esp_err.h"
#include "voice_cmd.h"

/* ============================================================
 * 语音功能选择（主菜单状态机）—— 语音模块的"接口"
 *
 * 语音模块 = 一个功能选择接口，通向三个功能：
 *
 *                 ┌──“蓝牙遥控”→ 蓝牙遥控模式（BT08B，接口占位）
 *   功能选择(默认) ─┼──“表情”   → 表情模式（TFT，接口占位）
 *                 └──“语音遥控”→ 语音遥控模式（前进/后退/左转/右转，已实现）
 *
 *   任意模式说“选择功能” → 刹停并返回功能选择。
 *
 * 职责：
 *   - 维护当前模式（app_mode_t）；
 *   - 按当前模式裁决 voice_cmd 上报的事件该给谁（路由/忽略）；
 *   - 切换模式时统一处理：刹停 → 旧模块 exit() → 新模块 enter() → 打印提示。
 * ============================================================ */

/** @brief 功能模式 */
typedef enum {
    APP_MODE_SELECT = 0,    /* 功能选择（默认：听 蓝牙遥控/攻城槌/语音遥控） */
    APP_MODE_BT,            /* 蓝牙遥控：手机 APP 控制（接口占位） */
    APP_MODE_RAM,           /* 攻城槌：语音进入，找二维码目标并反复撞击 */
    APP_MODE_VOICE,         /* 语音遥控：语音指挥小车（已实现） */
    APP_MODE_MAX,
} app_mode_t;

/**
 * @brief 初始化：motion_init + 三个功能模块 init，停在功能选择模式
 */
void app_menu_init(void);

/**
 * @brief 语音识别回调入口（注册给 voice_cmd_init）
 *
 * 按当前模式裁决事件：
 *   - 功能选择：蓝牙遥控/攻城槌/语音遥控 → 切模式；其余提示忽略
 *   - 蓝牙遥控 / 攻城槌：仅“选择功能”有效（返回菜单），其余提示忽略
 *   - 语音遥控：“选择功能”返回菜单；动作事件交给 voice_remote
 */
void app_menu_on_voice_evt(voice_evt_t evt, void *user_ctx);

/** @brief 当前模式 */
app_mode_t app_menu_get_mode(void);

/** @brief 模式名（打印/调试用） */
const char *app_menu_mode_name(app_mode_t mode);
