#include <stdbool.h>
#include "esp_log.h"
#include "esp_timer.h"

#include "motion.h"
#include "voice_cmd.h"
#include "voice_remote.h"

/* ============================================================
 * 语音遥控功能（已实现）：命令 → 定时动作 → 自动停
 *
 * 语义（2026-09-09 用户定稿）：
 *   每个动作词触发**固定时长**的动作，到时自动刹停，
 *   不需要再说“停止”（停止词保留作急刹）：
 *     前进/后退 —— 直行 VOICE_FWD_MS 后停
 *     左转/右转 —— 原地转 VOICE_TURN_MS 后停
 *   新动作词到来会覆盖旧动作（立即执行新的）。
 *
 * 方向（实车标定，与 task3-ble-control 一致）：
 *   turn 分量 **正=右转（顺时针）、负=左转**。
 *
 * ⚠️ 速度/时长参数按底盘实测标定：
 *    - VOICE_SPEED/VOICE_TURN_SPEED 为分量权重（motion.c 归一化
 *      到轮速上限），过大易打滑/过冲
 *    - VOICE_TURN_MS 以“想转的角度”实测为准
 * ============================================================ */

#define VOICE_SPEED      40      /* 直行权重（-100~100，正=前进） */
#define VOICE_TURN_SPEED 45      /* 原地旋转权重 */
#define VOICE_FWD_MS     1000    /* 前进/后退动作时长 ms */
#define VOICE_TURN_MS    500     /* 左转/右转动作时长 ms */

static const char *TAG = "voice_remote";
static bool s_active = false;
static esp_timer_handle_t s_stop_timer = NULL;

/* 到时自动刹停（esp_timer 任务上下文；motion_stop 非阻塞，安全） */
static void stop_timer_cb(void *arg)
{
    (void)arg;
    motion_stop();
    ESP_LOGI(TAG, "→ 动作完成，自动停止");
}

static void start_timed(int fwd, int turn, uint32_t ms, const char *name)
{
    ESP_LOGI(TAG, "→ %s（%lu ms）", name, (unsigned long)ms);
    motion_drive(fwd, turn, 0);
    /* 新动作覆盖旧定时：重启一次性定时器 */
    esp_timer_stop(s_stop_timer);
    esp_timer_start_once(s_stop_timer, (uint64_t)ms * 1000);
}

void voice_remote_init(void)
{
    s_active = false;
    if (s_stop_timer == NULL) {
        const esp_timer_create_args_t tcfg = {
            .callback = stop_timer_cb,
            .name = "voice_stop",
        };
        ESP_ERROR_CHECK(esp_timer_create(&tcfg, &s_stop_timer));
    }
    ESP_LOGI(TAG, "语音遥控模块初始化完成（权重：直行=%d 转向=%d；时长：直行=%dms 转向=%dms）",
             VOICE_SPEED, VOICE_TURN_SPEED, VOICE_FWD_MS, VOICE_TURN_MS);
}

void voice_remote_enter(void)
{
    s_active = true;
    motion_enable(true);        /* 语音遥控模式内电机使能 */
    motion_stop();
    ESP_LOGI(TAG, "★ 进入语音遥控：说“前进/后退/左转/右转”执行定时动作，自动停止");
}

void voice_remote_exit(void)
{
    s_active = false;
    esp_timer_stop(s_stop_timer);
    motion_stop();
    motion_enable(false);       /* 离开模式立即断电保护 */
    ESP_LOGI(TAG, "已退出语音遥控（返回功能选择）");
}

void voice_remote_execute(voice_evt_t evt)
{
    switch (evt) {
    case VOICE_EVT_DRIVE_FORWARD:
        start_timed(VOICE_SPEED, 0, VOICE_FWD_MS, "前进");
        break;
    case VOICE_EVT_DRIVE_BACKWARD:
        start_timed(-VOICE_SPEED, 0, VOICE_FWD_MS, "后退");
        break;
    case VOICE_EVT_DRIVE_LEFT:
        /* 左转 = turn 负方向（实车标定：正=右转） */
        start_timed(0, -VOICE_TURN_SPEED, VOICE_TURN_MS, "左转");
        break;
    case VOICE_EVT_DRIVE_RIGHT:
        start_timed(0, VOICE_TURN_SPEED, VOICE_TURN_MS, "右转");
        break;
    case VOICE_EVT_DRIVE_STOP:
        ESP_LOGI(TAG, "→ 停止（急刹）");
        esp_timer_stop(s_stop_timer);
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
