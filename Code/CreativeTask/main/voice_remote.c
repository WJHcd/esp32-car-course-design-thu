#include <stdbool.h>
#include "esp_log.h"

#include "motion.h"
#include "voice_cmd.h"
#include "voice_remote.h"

/* ============================================================
 * 语音遥控功能（已实现）：命令 → motion_drive(f/t/s)
 *
 * 语义：前进/后退/左转/右转均为“持续动作”，
 *   直到收到新命令、停止、或返回功能选择（app_menu 会先刹停）。
 * 所有动作调用都在 voice_detect 任务上下文，motion_* 非阻塞。
 *
 * ⚠️ 速度参数为相对权重（-100~100），motion.c 会把峰值归一化到
 *    实车标定轮速（单轴满量程 ≈ 30% 轮速）。语音演示默认
 *    直行/转向都给足权重；如觉过快/过慢，调 VOICE_SPEED/
 *    VOICE_TURN_SPEED 并实车验证（方向为负号反转即可）。
 * ============================================================ */

#define VOICE_SPEED      80      /* 直行权重（正=前进） */
#define VOICE_TURN_SPEED 80      /* 原地旋转权重（正=逆时针） */

static const char *TAG = "voice_remote";
static bool s_active = false;

void voice_remote_init(void)
{
    s_active = false;
    ESP_LOGI(TAG, "语音遥控模块初始化完成（权重：直行=%d 转向=%d）",
             VOICE_SPEED, VOICE_TURN_SPEED);
}

void voice_remote_enter(void)
{
    s_active = true;
    motion_enable(true);        /* 语音遥控模式内电机使能 */
    motion_stop();
    ESP_LOGI(TAG, "★ 进入语音遥控：说“前进/后退/左转/右转/停止”指挥小车");
}

void voice_remote_exit(void)
{
    s_active = false;
    motion_stop();
    motion_enable(false);       /* 离开模式立即断电保护 */
    ESP_LOGI(TAG, "已退出语音遥控（返回功能选择）");
}

void voice_remote_execute(voice_evt_t evt)
{
    switch (evt) {
    case VOICE_EVT_DRIVE_FORWARD:
        ESP_LOGI(TAG, "→ 前进（持续）");
        motion_drive(VOICE_SPEED, 0, 0);
        break;
    case VOICE_EVT_DRIVE_BACKWARD:
        ESP_LOGI(TAG, "→ 后退（持续）");
        motion_drive(-VOICE_SPEED, 0, 0);
        break;
    case VOICE_EVT_DRIVE_LEFT:
        ESP_LOGI(TAG, "→ 左转（原地，持续）");
        motion_drive(0, VOICE_TURN_SPEED, 0);
        break;
    case VOICE_EVT_DRIVE_RIGHT:
        ESP_LOGI(TAG, "→ 右转（原地，持续）");
        motion_drive(0, -VOICE_TURN_SPEED, 0);
        break;
    case VOICE_EVT_DRIVE_STOP:
        ESP_LOGI(TAG, "→ 停止");
        motion_stop();
        break;
    default:
        /* 菜单词/其他模式词在语音遥控模式下无效（路由由 app_menu 负责） */
        ESP_LOGW(TAG, "语音遥控忽略“%s”：仅支持 前进/后退/左转/右转/停止",
                 voice_evt_to_str(evt));
        break;
    }
}

bool voice_remote_is_active(void)
{
    return s_active;
}
