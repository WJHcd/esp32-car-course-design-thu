#include <stdbool.h>
#include "esp_log.h"

#include "motion.h"
#include "expression.h"

/* ============================================================
 * 表情功能 —— 【接口占位，未接入 TFT 彩屏】
 *
 * 当前仅完成接口与模式状态，不驱动屏幕（打印提示）。
 * 实现 TODO（接入 ST7735 时替换本文件，参考 feature/qr 分支
 * Code/QR 工程的 display.c / tft 驱动）：
 *
 *   1. TFT 引脚（LQ_TFT18SPIV33 1.8" 128x160，SPI 只写）：
 *        SCK=21 SDI=18 CS=42 DC=41 RST=40（同 QR 工程）；
 *        ⚠️ 与麦克风(4/5/6)、电机(1/2/4~11) 冲突，整机接线需统一避让；
 *   2. expression_enter() 中：display_init() + display_idle()
 *        （或表情轮播任务，按帧切换 EXPR_FACE_*）；
 *   3. expression_set_face() → display_face(face)；
 *   4. expression_exit() 中：关屏/停止动画任务。
 * 表情位图与解码方式（esp_jpeg）同 QR 工程 display.c。
 * ============================================================ */

static const char *TAG = "expression";
static bool s_active = false;

static const char *s_face_names[EXPR_FACE_MAX] = {
    "笑脸", "哭脸", "惊讶", "眨眼",
};

esp_err_t expression_init(void)
{
    s_active = false;
    ESP_LOGI(TAG, "表情模块初始化完成（占位：TFT 未接入）");
    return ESP_OK;
}

esp_err_t expression_enter(void)
{
    motion_stop();
    s_active = true;
    ESP_LOGI(TAG, "★ 进入表情功能：小车开始展示表情");
    ESP_LOGI(TAG, "  [占位] 尚未接入 ST7735 彩屏——接口已留出，见 expression.c 顶部 TODO");
    ESP_LOGI(TAG, "  语音仅响应“选择功能”返回菜单");
    return ESP_OK;
}

void expression_exit(void)
{
    s_active = false;
    motion_stop();
    ESP_LOGI(TAG, "已退出表情功能（返回功能选择）");
}

esp_err_t expression_set_face(expression_face_t face)
{
    if (face >= EXPR_FACE_MAX) {
        ESP_LOGW(TAG, "无效表情编号: %d", face);
        return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "[占位] 显示表情：%s", s_face_names[face]);
    return ESP_OK;
}

bool expression_is_active(void)
{
    return s_active;
}
