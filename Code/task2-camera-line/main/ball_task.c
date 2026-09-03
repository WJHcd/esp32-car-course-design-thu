#include "ball_task.h"
#include "gimbal.h"
#include "walking.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * 推球状态机实现
 *
 * 视觉约定（来自 ball_detect）：
 *   - 黑球/蓝球：ball_field 里各自的 detected + x_ratio + area_ratio；
 *   - 红角标：red_marker[0..count-1]，按面积从大到小；
 *
 * 推球策略（v1，简化机动）：
 *   - 车对准球后直接前顶，球在车与目标角之间时自然被推向角标；
 *   - 推进中转向目标 = 可见的红角标（朝角标修正），无角标可见时
 *     修正目标 = 球本身（保持球居中）；
 *   - 到位判定：球与某红角标在画面里的水平距离足够近；
 *   - 球贴近车头时会滑出视野：盲走一小段，随后靠 AIM/PUSH 的
 *     新帧逻辑重新捕获去向。
 *
 * 时间工具：walking.c 的 elapsed_ms 是 static，这里自带同款实现
 * （uint32_t 减法天然处理回绕）。
 * ============================================================ */

static const char *TAG = "ball_task";

static const char *s_state_name[] = {
    "GIMBAL_UP", "SURVEY", "AIM", "PUSH", "DONE",
};

typedef enum {
    TARGET_BLACK = 0,
    TARGET_BLUE,
    TARGET_COUNT,
} target_color_t;

/* 扫视方向：+1 = 顺时针先转（ balls 在正前方时最先扫到） */
#define BALL_SURVEY_DIR         (+1)

static ball_task_state_t s_state;
static uint32_t s_state_enter_ms;
static uint32_t s_total_start_ms;
static uint32_t s_last_frame_seq;
static uint32_t s_last_ball_seen_ms;

static inline uint32_t since_ms(uint32_t now_ms, uint32_t since_ms_)
{
    return now_ms - since_ms_;
}

/* 扫视期间的双球观测（确认帧数 + 最近一次方位/面积） */
typedef struct {
    int confirm;
    float x_ratio;
    float area_ratio;
} ball_sighting_t;

static ball_sighting_t s_seen[TARGET_COUNT];
static bool s_pending[TARGET_COUNT];        /* 该球尚未推送 */
static target_color_t s_current;            /* 当前推送的球 */

static const char *s_color_name[TARGET_COUNT] = { "黑球", "蓝球" };

static const ball_blob_t *pick_target_blob(const ball_field_result_t *b)
{
    return (s_current == TARGET_BLACK) ? &b->black_ball : &b->blue_ball;
}

static void set_state(ball_task_state_t st, uint32_t now_ms)
{
    s_state = st;
    s_state_enter_ms = now_ms;
    if (st == BALL_TASK_PUSH) {
        s_last_ball_seen_ms = now_ms;
    }
    ESP_LOGI(TAG, "→ %s", s_state_name[st]);
}

void ball_task_start(uint32_t now_ms)
{
    s_state = BALL_TASK_GIMBAL_UP;
    s_state_enter_ms = now_ms;
    s_total_start_ms = now_ms;
    s_last_frame_seq = 0;
    s_last_ball_seen_ms = now_ms;
    memset(s_seen, 0, sizeof(s_seen));
    s_pending[TARGET_BLACK] = true;
    s_pending[TARGET_BLUE] = true;
    gimbal_set_tilt(BALL_TASK_TILT_SURVEY_DEG);   /* 抬云台获得俯视视角 */
    ESP_LOGI(TAG, "找球任务启动：云台抬至 %d°", BALL_TASK_TILT_SURVEY_DEG);
}

void ball_task_reset(uint32_t now_ms)
{
    (void)now_ms;
    s_state = BALL_TASK_GIMBAL_UP;
    s_state_enter_ms = 0;
    s_total_start_ms = 0;
    s_last_frame_seq = 0;
    s_last_ball_seen_ms = 0;
    memset(s_seen, 0, sizeof(s_seen));
    s_pending[TARGET_BLACK] = false;
    s_pending[TARGET_BLUE] = false;
}

bool ball_task_is_done(void)
{
    return s_state == BALL_TASK_DONE;
}

const char *ball_task_state_name(void)
{
    if ((int)s_state < 0 ||
        (int)s_state >= (int)(sizeof(s_state_name) / sizeof(s_state_name[0]))) {
        return "?";
    }
    return s_state_name[s_state];
}

/* 选当前该推的球：两个都待推送时取面积大的（更近） */
static bool choose_next_target(void)
{
    if (s_pending[TARGET_BLACK] && s_pending[TARGET_BLUE]) {
        s_current = (s_seen[TARGET_BLACK].area_ratio >=
                     s_seen[TARGET_BLUE].area_ratio) ? TARGET_BLACK : TARGET_BLUE;
    } else if (s_pending[TARGET_BLACK]) {
        s_current = TARGET_BLACK;
    } else if (s_pending[TARGET_BLUE]) {
        s_current = TARGET_BLUE;
    } else {
        return false;
    }
    ESP_LOGI(TAG, "选中%s（area=%.3f x=%+.2f）", s_color_name[s_current],
             s_seen[s_current].area_ratio, s_seen[s_current].x_ratio);
    return true;
}

/* 全任务兜底超时 */
static bool task_timed_out(uint32_t now_ms)
{
    return since_ms(now_ms, s_total_start_ms) >= BALL_TASK_TOTAL_MS;
}

void ball_task_update(const camera_line_result_t *cam,
                      const ball_field_result_t *balls, uint32_t now_ms)
{
    const bool new_frame = (cam->frame_seq != s_last_frame_seq);
    if (new_frame) {
        s_last_frame_seq = cam->frame_seq;
    }

    if (task_timed_out(now_ms)) {
        walking_direct_command(0, 0, 0);
        ESP_LOGW(TAG, "找球任务总超时，结束");
        set_state(BALL_TASK_DONE, now_ms);
        return;
    }

    switch (s_state) {
    case BALL_TASK_GIMBAL_UP:
        walking_direct_command(0, 0, 0);   /* 静置让云台到位 */
        if (since_ms(now_ms, s_state_enter_ms) >= BALL_GIMBAL_DWELL_MS) {
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;

    case BALL_TASK_SURVEY: {
        /* 原地慢转扫视；每个新帧更新双球观测 */
        if (new_frame) {
            if (balls->black_ball.detected) {
                s_seen[TARGET_BLACK].confirm++;
                s_seen[TARGET_BLACK].x_ratio = balls->black_ball.x_ratio;
                s_seen[TARGET_BLACK].area_ratio = balls->black_ball.area_ratio;
            }
            if (balls->blue_ball.detected) {
                s_seen[TARGET_BLUE].confirm++;
                s_seen[TARGET_BLUE].x_ratio = balls->blue_ball.x_ratio;
                s_seen[TARGET_BLUE].area_ratio = balls->blue_ball.area_ratio;
            }

            const bool black_ok = !s_pending[TARGET_BLACK] ||
                                  s_seen[TARGET_BLACK].confirm >= BALL_SURVEY_CONFIRM;
            const bool blue_ok = !s_pending[TARGET_BLUE] ||
                                 s_seen[TARGET_BLUE].confirm >= BALL_SURVEY_CONFIRM;
            if (black_ok && blue_ok) {
                walking_direct_command(0, 0, 0);
                if (choose_next_target()) {
                    set_state(BALL_TASK_AIM, now_ms);
                } else {
                    set_state(BALL_TASK_DONE, now_ms);   /* 没有待推的球 */
                }
                break;
            }
        }
        walking_direct_command(0, BALL_SURVEY_DIR * BALL_SURVEY_TURN_PCT, 0);

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_SURVEY_TIMEOUT_MS) {
            /* 扫视超时：转够一圈了还没集齐——按已见的目标继续 */
            walking_direct_command(0, 0, 0);
            if (choose_next_target()) {
                set_state(BALL_TASK_AIM, now_ms);
            } else {
                ESP_LOGW(TAG, "扫视超时且未发现任何球，任务结束");
                set_state(BALL_TASK_DONE, now_ms);
            }
        }
        break;
    }

    case BALL_TASK_AIM: {
        const ball_blob_t *ball = pick_target_blob(balls);
        if (new_frame && ball->detected) {
            s_seen[s_current].x_ratio = ball->x_ratio;   /* 刷新方位 */
            const float off = ball->x_ratio - 0.5f;
            if (fabsf(off) <= BALL_AIM_DEADBAND) {
                set_state(BALL_TASK_PUSH, now_ms);
                break;
            }
            /* 球偏哪边就往哪边原地转（按偏差比例转，越接近转越慢） */
            const int turn = (int)(off * BALL_PUSH_STEER_GAIN * 2.0f);
            walking_direct_command(0, turn, 0);
        } else {
            walking_direct_command(0, 0, 0);   /* 等下一帧，不盲转 */
        }

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_AIM_TIMEOUT_MS) {
            ESP_LOGW(TAG, "对准 %s 超时，重新扫视", s_color_name[s_current]);
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;
    }

    case BALL_TASK_PUSH: {
        const ball_blob_t *ball = pick_target_blob(balls);
        float steer_target = 0.5f;   /* 无参考时按画面正中直行 */
        bool have_new_ball = false;

        if (new_frame && ball->detected) {
            s_seen[s_current].x_ratio = ball->x_ratio;
            have_new_ball = true;
            s_last_ball_seen_ms = now_ms;

            /* 转向目标：可见的红角标里选水平位置最接近球的那个 */
            const ball_blob_t *best_marker = NULL;
            float best_dist = 1e9f;
            for (int i = 0; i < balls->red_marker_count; i++) {
                const float d = fabsf(balls->red_marker[i].x_ratio - ball->x_ratio);
                if (d < best_dist) {
                    best_dist = d;
                    best_marker = &balls->red_marker[i];
                }
            }
            if (best_marker != NULL) {
                if (best_dist <= BALL_MARKER_ADJACENT_ERR) {
                    /* 球贴上角标：到位 */
                    walking_direct_command(0, 0, 0);
                    s_pending[s_current] = false;
                    ESP_LOGI(TAG, "%s 已推到角标（dist=%.2f），%s",
                             s_color_name[s_current], (double)best_dist,
                             (s_pending[TARGET_BLACK] || s_pending[TARGET_BLUE])
                                 ? "处理下一球" : "双球完成");
                    if (s_pending[TARGET_BLACK] || s_pending[TARGET_BLUE]) {
                        set_state(BALL_TASK_SURVEY, now_ms);
                    } else {
                        set_state(BALL_TASK_DONE, now_ms);
                    }
                    break;
                }
                steer_target = best_marker->x_ratio;   /* 朝角标方向推 */
            }
        }

        if (have_new_ball) {
            /* 差速修正：让转向目标保持在画面中央（车头顶着球与角标连线） */
            const float off = steer_target - 0.5f;
            const int turn = (int)(off * BALL_PUSH_STEER_GAIN);
            walking_direct_command(BALL_PUSH_SPEED_PCT, turn, 0);
        } else if (since_ms(now_ms, s_last_ball_seen_ms) <= BALL_LOST_BLIND_MS) {
            /* 球贴近车头滑出视野：只在限定时间内短暂盲走。 */
            walking_direct_command(BALL_PUSH_SPEED_PCT, 0, 0);
        } else {
            /* 原代码虽定义 BALL_LOST_BLIND_MS，却会一直前进到 15s 超时。
             * 超过盲走窗口后立即停车，回 AIM 等待重新捕获当前球。 */
            walking_direct_command(0, 0, 0);
            ESP_LOGW(TAG, "%s 丢失超过 %u ms，停止并重新对准",
                     s_color_name[s_current], (unsigned)BALL_LOST_BLIND_MS);
            set_state(BALL_TASK_AIM, now_ms);
            break;
        }

        /* 单球推球超时：视为完成该球的尝试，换下一球 */
        if (since_ms(now_ms, s_state_enter_ms) >= BALL_PUSH_TIMEOUT_MS) {
            walking_direct_command(0, 0, 0);
            s_pending[s_current] = false;
            ESP_LOGW(TAG, "%s 推球超时，放弃该球", s_color_name[s_current]);
            if (s_pending[TARGET_BLACK] || s_pending[TARGET_BLUE]) {
                set_state(BALL_TASK_SURVEY, now_ms);
            } else {
                set_state(BALL_TASK_DONE, now_ms);
            }
        }
        break;
    }

    case BALL_TASK_DONE:
    default:
        walking_direct_command(0, 0, 0);
        break;
    }
}
