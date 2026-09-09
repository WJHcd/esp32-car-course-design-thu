#pragma once
#include "esp_err.h"

/* ============================================================
 * 表情功能 —— 【接口占位，待接入 TFT 彩屏】
 *
 * 设计：进入表情功能后，小车 TFT 屏幕轮播/点播表情。
 * 表情编号与 Code/QR 工程（feature/qr 分支，ST7735 1.8" 屏）
 * 的 display_face() 一致：
 *   0 = 笑脸   1 = 哭脸   2 = 惊讶   3 = 眨眼
 *
 * 留出的接口：
 *   init / enter / exit 由 app_menu 模式切换时调用，其余模块无需改动。
 *   后续实现只需替换 expression.c：
 *     1. 移植 ST7735 驱动 + 表情位图（TFT 引脚 21/18/42/41/40 同 QR 工程，
 *        注意与麦克风/电机引脚避让）；
 *     2. enter() 里显示默认表情（可带呼吸/轮播动画任务）；
 *     3. 需要语音点播表情时：在 voice_cmd.c 词表加词 + app_menu 转发事件
 *        到 expression_set_face()，重新烧录即可（模型无需改动）。
 * ============================================================ */

/** @brief 表情编号（与 QR 工程 display_face 对齐） */
typedef enum {
    EXPR_FACE_SMILE = 0,    /* 笑脸 */
    EXPR_FACE_CRY,          /* 哭脸 */
    EXPR_FACE_SURPRISE,     /* 惊讶 */
    EXPR_FACE_BLINK,        /* 眨眼 */
    EXPR_FACE_MAX,
} expression_face_t;

/**
 * @brief 初始化表情模块（占位：仅记录状态）
 */
esp_err_t expression_init(void);

/**
 * @brief 进入表情模式（app_menu 切到 APP_MODE_FACE 时调用）
 *
 * 占位实现：打印提示 + 刹停。TODO: 点亮 TFT 显示默认表情。
 */
esp_err_t expression_enter(void);

/**
 * @brief 退出表情模式（说"选择功能"返回菜单时调用）
 *
 * 占位实现：刹停。TODO: 关闭显示/停止动画任务。
 */
void expression_exit(void);

/**
 * @brief 显示指定表情（语音点播/TFT 轮播入口）
 * @param face 表情编号，见 expression_face_t
 *
 * 占位实现：仅打印。TODO: 调 display_face(face)。
 */
esp_err_t expression_set_face(expression_face_t face);

/** @brief 当前是否处于表情模式（供显示任务/测试判断） */
bool expression_is_active(void);
