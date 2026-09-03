#include "ball_task.h"
#include "gimbal.h"
#include "walking.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * 推球状态机实现（v3 三点一线法）
 *
 * 视觉约定（来自 ball_detect，2026-09-03 黑球→红球、角标→绿色）：
 *   - 红球/蓝球：ball_field 里各自的 detected + x_ratio + area_ratio；
 *   - 角标（绿）：marker[0..count-1]，按面积从大到小；
 *   - 云台抬至 BALL_TASK_TILT_SURVEY_DEG 后全程固定。
 *
 * 三点一线的图像几何（ALIGN 的依据）：
 *   车在 C、球在 B、角标在 M（M 比 B 远）。设航向为参考方向，到 B 的
 *   方位角 β、到 M 的方位角 μ。C/B/M 共线（B 在中间）⇔ β=μ ⇔ 画面中
 *   球与角标的 x 重合，即 e = x球 − x角标 = 0。
 *   关键性质：纯旋转同幅移动两条方位线，e 不变；平移才改变 e。
 *   实例（d球=1m β=+5°、d角标=2m μ=−5°）：e>0（球偏右）时连线在车
 *   右侧约 0.35m → 车向右横移，两方位角同步左移但球移得更快，横移
 *   0.35m 后 β=μ=−14.8°，e 归零。故对准动作 = 横移消差 + 踢转居中
 *   （旋转只改目标的公共 x，不破坏已形成的共线）。
 *
 * 推球：对准后直线撞球，球自行滚向角标；推进中持续微修 e 与居中；
 *   到位判定 = 同帧球+角标水平 x、纵向 y（≈地面深度）都贴近 + 静止
 *   两帧确认。完成后按累计推进时长盲退回近似原位，再推下一球。
 *
 * 时间工具：walking.c 的 elapsed_ms 是 static，这里自带同款实现
 * （uint32_t 减法天然处理回绕）。
 * ============================================================ */

static const char *TAG = "ball_task";

static const char *s_state_name[] = {
    "GIMBAL_UP", "SURVEY", "APPROACH", "ALIGN", "PUSH", "RETURN", "DONE",
};

typedef enum {
    TARGET_RED = 0,
    TARGET_BLUE,
    TARGET_COUNT,
} target_color_t;

/* 扫视方向（SPIN_ENABLE=1 时使用）：+1 = 顺时针先转 */
#define BALL_SURVEY_DIR         (+1)

static ball_task_state_t s_state;
static uint32_t s_state_enter_ms;
static uint32_t s_total_start_ms;
static uint32_t s_last_frame_seq;
static uint32_t s_last_ball_seen_ms;

/* 对准/推球/退回状态（进状态时在 set_state 里复位） */
static uint32_t s_kick_until_ms;         /* 本次踢转结束时刻（0=无踢转） */
static int8_t s_kick_dir;                /* +1=右转 / -1=左转 */
static float s_align_strafe;             /* 对准横移档（脉冲期间生效） */
static uint32_t s_strafe_until_ms;       /* 本次横移脉冲结束时刻（0=无） */
static uint32_t s_push_done_pending_ms;  /* 0 = 无待确认；非 0 = 到位待静止确认 */
static uint32_t s_push_launch_start_ms;  /* 起步强推起点（0=无） */
static uint32_t s_push_drive_ms;         /* PUSH 中前进指令累计时长（退回用） */
static float s_push_steer_target;        /* 最近一次生效转向目标（盲区续用） */
static uint32_t s_return_ms;             /* 本次退回计划时长 */

static inline uint32_t since_ms(uint32_t now_ms, uint32_t since_ms_)
{
    return now_ms - since_ms_;
}

/* 扫视/推进期间的双球观测（确认帧数 + 最近一次方位/面积/纵位） */
typedef struct {
    int confirm;
    float x_ratio;
    float y_ratio;
    float area_ratio;
} ball_sighting_t;

static ball_sighting_t s_seen[TARGET_COUNT];
static bool s_pending[TARGET_COUNT];        /* 该球尚未推送 */
static target_color_t s_current;            /* 当前推送的球 */

static const char *s_color_name[TARGET_COUNT] = { "红球", "蓝球" };

static const ball_blob_t *pick_target_blob(const ball_field_result_t *b)
{
    return (s_current == TARGET_RED) ? &b->red_ball : &b->blue_ball;
}

/* 从可见角标里选水平位置最接近参考 x（球心）的那个；
 * 返回 NULL = 本帧无角标可见。 */
static const ball_blob_t *pick_nearest_marker(const ball_field_result_t *b,
                                              float ref_x, float *out_dist)
{
    const ball_blob_t *best = NULL;
    float best_dist = 1e9f;
    for (int i = 0; i < b->marker_count; i++) {
        const float d = fabsf(b->marker[i].x_ratio - ref_x);
        if (d < best_dist) {
            best_dist = d;
            best = &b->marker[i];
        }
    }
    if (out_dist) *out_dist = best_dist;
    return best;
}

/* PUSH 当前推进速度：强推 20%×350ms → 600ms 线性缓降至巡航 11%。
 * 台阶式掉速会在死区上沿闷熄（车刚动起来就掉进 <9% 出力区）。 */
static float push_speed_now(uint32_t now_ms)
{
    const uint32_t t = now_ms - s_push_launch_start_ms;   /* 回绕安全 */
    if (s_push_launch_start_ms == 0 ||
        t >= BALL_PUSH_LAUNCH_MS + BALL_PUSH_RAMP_MS) {
        return BALL_PUSH_SPEED_PCT;
    }
    if (t < BALL_PUSH_LAUNCH_MS) {
        return BALL_PUSH_LAUNCH_PCT;
    }
    const float k = (float)(t - BALL_PUSH_LAUNCH_MS) / (float)BALL_PUSH_RAMP_MS;
    return BALL_PUSH_LAUNCH_PCT -
           (BALL_PUSH_LAUNCH_PCT - BALL_PUSH_SPEED_PCT) * k;
}

/* 按偏差调度一次踢转（"踢一脚-看一帧"：时长随偏差缩小，越近越小步）。
 * off>0 = 目标在画面右侧 → 往右踢（+1）。 */
static void schedule_kick(float off, uint32_t now_ms)
{
    uint32_t kick = (uint32_t)(fabsf(off) * BALL_ALIGN_KICK_SCALE_MS);
    if (kick < BALL_ALIGN_KICK_MIN_MS) kick = BALL_ALIGN_KICK_MIN_MS;
    if (kick > BALL_ALIGN_KICK_MAX_MS) kick = BALL_ALIGN_KICK_MAX_MS;
    s_kick_until_ms = now_ms + kick;
    s_kick_dir = (off > 0.0f) ? +1 : -1;
}

/* 按偏差调度一次横移脉冲（"移一脚-看一帧"：时长随 |e| 缩小，收敛线
 * 附近步距细，避免帧间连续横移的过冲；方向同踢转约定）。 */
static void schedule_strafe(float e, uint32_t now_ms)
{
    uint32_t pulse = (uint32_t)(fabsf(e) * BALL_ALIGN_STRAFE_SCALE_MS);
    if (pulse < BALL_ALIGN_STRAFE_MIN_MS) pulse = BALL_ALIGN_STRAFE_MIN_MS;
    if (pulse > BALL_ALIGN_STRAFE_MAX_MS) pulse = BALL_ALIGN_STRAFE_MAX_MS;
    s_strafe_until_ms = now_ms + pulse;
    s_align_strafe = -BALL_ALIGN_STRAFE_SIGN *
                     (e > 0.0f ? BALL_ALIGN_STRAFE_PCT : -BALL_ALIGN_STRAFE_PCT);
}

static void set_state(ball_task_state_t st, uint32_t now_ms)
{
    s_state = st;
    s_state_enter_ms = now_ms;
    /* 云台全程固定在扫视角（v3 起不再远近切换；重复设置无害） */
    gimbal_set_tilt(BALL_TASK_TILT_SURVEY_DEG);

    if (st == BALL_TASK_PUSH) {
        s_last_ball_seen_ms = now_ms;
        s_push_done_pending_ms = 0;
        s_push_launch_start_ms = now_ms;   /* 起步强推→缓降 */
        s_push_drive_ms = 0;
        s_push_steer_target = 0.5f;
    } else if (st == BALL_TASK_APPROACH) {
        /* 靠近起步同样走强推→缓降（远距离起步也受死区闷熄困扰） */
        s_last_ball_seen_ms = now_ms;
        s_push_launch_start_ms = now_ms;
    } else if (st == BALL_TASK_RETURN) {
        /* 退回时长 = 推进累计时长 × 系数，上限兜底 */
        uint32_t need = (uint32_t)((float)s_push_drive_ms * BALL_RETURN_TIME_SCALE);
        if (need < BALL_ALIGN_KICK_MIN_MS) need = BALL_ALIGN_KICK_MIN_MS;
        if (need > BALL_RETURN_MAX_MS) need = BALL_RETURN_MAX_MS;
        s_return_ms = need;
        ESP_LOGI(TAG, "退回原位：盲退 %lu ms（推进累计 %lu ms）",
                 (unsigned long)s_return_ms, (unsigned long)s_push_drive_ms);
    } else if (st == BALL_TASK_ALIGN) {
        s_kick_until_ms = 0;       /* 等第一帧评估后再动作 */
        s_kick_dir = 0;
        s_align_strafe = 0;
        s_strafe_until_ms = 0;
        s_last_ball_seen_ms = now_ms;   /* 无球等待窗从进对准起算 */
    } else if (st == BALL_TASK_SURVEY) {
        /* 每轮等球重新计数确认帧——推完一球/对准超时回来时若沿用旧
         * 计数，会拿旧方位直接进对准 */
        for (int i = 0; i < TARGET_COUNT; i++) {
            s_seen[i].confirm = 0;
        }
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
    s_pending[TARGET_RED] = true;
    s_pending[TARGET_BLUE] = true;
    gimbal_set_tilt(BALL_TASK_TILT_SURVEY_DEG);   /* 抬云台获得俯视视角 */
    ESP_LOGI(TAG, "找球任务启动：云台抬至 %d°（全程固定）",
             BALL_TASK_TILT_SURVEY_DEG);
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
    s_pending[TARGET_RED] = false;
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

/* 选当前该推的球：只在"本轮等球见过"（confirm≥1）的待推球里选，
 * 两个都可见时取面积大的（更近）；本轮都没见过返回 false */
static bool choose_next_target(void)
{
    const bool red_avail = s_pending[TARGET_RED] &&
                           s_seen[TARGET_RED].confirm >= 1;
    const bool blue_avail = s_pending[TARGET_BLUE] &&
                            s_seen[TARGET_BLUE].confirm >= 1;
    if (red_avail && blue_avail) {
        s_current = (s_seen[TARGET_RED].area_ratio >=
                     s_seen[TARGET_BLUE].area_ratio) ? TARGET_RED : TARGET_BLUE;
    } else if (red_avail) {
        s_current = TARGET_RED;
    } else if (blue_avail) {
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

/* 该球完成（或放弃）后的去向：还有待推球 → 退回原位再等下一球 */
static void after_ball_finished(uint32_t now_ms)
{
    if (s_pending[TARGET_RED] || s_pending[TARGET_BLUE]) {
        set_state(BALL_TASK_RETURN, now_ms);
    } else {
        set_state(BALL_TASK_DONE, now_ms);
    }
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
#if BALL_SURVEY_SPIN_ENABLE
        /* 间歇扫视（正式赛道自主搜索）：短转一档 → 静止看帧。
         * 观测只在静止相位更新，旋转相位的拖影帧不参与确认。 */
        const uint32_t cycle_ms = BALL_SURVEY_BURST_MS + BALL_SURVEY_HOLD_MS;
        const uint32_t phase_in = since_ms(now_ms, s_state_enter_ms) % cycle_ms;
        const bool rotating = phase_in < BALL_SURVEY_BURST_MS;
        const bool creeping = false;
#else
        /* 静止等摆球；静置期过后仍无待推球确认 → 缓慢向前蠕进找球
         * （边走边确认，走到球入视野确认即停转 APPROACH） */
        const bool rotating = false;
        const bool creeping =
            since_ms(now_ms, s_state_enter_ms) >= BALL_SURVEY_CREEP_DELAY_MS;
#endif
        if (new_frame && !rotating) {
            if (balls->red_ball.detected) {
                s_seen[TARGET_RED].confirm++;
                s_seen[TARGET_RED].x_ratio = balls->red_ball.x_ratio;
                s_seen[TARGET_RED].area_ratio = balls->red_ball.area_ratio;
            }
            if (balls->blue_ball.detected) {
                s_seen[TARGET_BLUE].confirm++;
                s_seen[TARGET_BLUE].x_ratio = balls->blue_ball.x_ratio;
                s_seen[TARGET_BLUE].area_ratio = balls->blue_ball.area_ratio;
            }

            const bool red_ok = !s_pending[TARGET_RED] ||
                                s_seen[TARGET_RED].confirm >= BALL_SURVEY_CONFIRM;
            const bool blue_ok = !s_pending[TARGET_BLUE] ||
                                 s_seen[TARGET_BLUE].confirm >= BALL_SURVEY_CONFIRM;

            if (red_ok && blue_ok) {
                /* 双球都确认：按面积选近的先推 */
                walking_direct_command(0, 0, 0);
                if (choose_next_target()) {
                    set_state(BALL_TASK_APPROACH, now_ms);
                } else {
                    set_state(BALL_TASK_DONE, now_ms);   /* 没有待推的球 */
                }
                break;
            }

            /* 只确认到一个待推球：直接推它，推完回 SURVEY 再等另一个 */
            target_color_t ready = TARGET_COUNT;
            if (s_pending[TARGET_RED] &&
                s_seen[TARGET_RED].confirm >= BALL_SURVEY_CONFIRM) {
                ready = TARGET_RED;
            } else if (s_pending[TARGET_BLUE] &&
                       s_seen[TARGET_BLUE].confirm >= BALL_SURVEY_CONFIRM) {
                ready = TARGET_BLUE;
            }
            if (ready != TARGET_COUNT) {
                walking_direct_command(0, 0, 0);
                s_current = ready;
                ESP_LOGI(TAG, "本轮仅确认到%s，先推它",
                         s_color_name[s_current]);
                set_state(BALL_TASK_APPROACH, now_ms);
                break;
            }
        }
        walking_direct_command(
#if BALL_SURVEY_SPIN_ENABLE
            0, rotating ? BALL_SURVEY_DIR * BALL_SURVEY_TURN_PCT : 0,
#else
            creeping ? BALL_SURVEY_CREEP_PCT : 0, 0,
#endif
            0);

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_SURVEY_TIMEOUT_MS) {
            /* 等待超时：按已见的目标继续，没见过任何球则结束 */
            walking_direct_command(0, 0, 0);
            if (choose_next_target()) {
                set_state(BALL_TASK_APPROACH, now_ms);
            } else {
                ESP_LOGW(TAG, "等待超时且未发现任何球，任务结束");
                set_state(BALL_TASK_DONE, now_ms);
            }
        }
        break;
    }

    case BALL_TASK_APPROACH: {
        /* 朝选定球开进：远处球勉强可辨、角标不可见，先靠近到球占画面
         * 偏底部（车到球近前，角标大概率同时入视野）再做三点一线。
         * 转向沿用推进的差速修正（有前进分量，小转向可落地）；起步走
         * 强推→缓降（复用 push_speed_now，见 set_state 的 APPROACH 分支）。 */
        const ball_blob_t *ball = pick_target_blob(balls);

        if (new_frame) {
            if (ball->detected) {
                s_seen[s_current].x_ratio = ball->x_ratio;
                s_seen[s_current].y_ratio = ball->y_ratio;
                s_seen[s_current].area_ratio = ball->area_ratio;
                s_last_ball_seen_ms = now_ms;

                if (ball->y_ratio >= BALL_APPROACH_Y_TARGET) {
                    walking_direct_command(0, 0, 0);
                    ESP_LOGI(TAG, "%s 已靠近（y=%.2f），开始三点一线对准",
                             s_color_name[s_current], (double)ball->y_ratio);
                    set_state(BALL_TASK_ALIGN, now_ms);
                    break;
                }
            } else if (s_seen[s_current].y_ratio >= BALL_APPROACH_Y_ARRIVE_LOST) {
                /* 球滑出画面底部 = 贴到车头：视为已到位 */
                walking_direct_command(0, 0, 0);
                ESP_LOGI(TAG, "%s 出画（最后 y=%.2f），视为已靠近，开始对准",
                         s_color_name[s_current],
                         (double)s_seen[s_current].y_ratio);
                set_state(BALL_TASK_ALIGN, now_ms);
                break;
            } else if (since_ms(now_ms, s_last_ball_seen_ms) >
                       BALL_PUSH_BLIND_MS) {
                /* 球丢得早（最后位置还高）：可能跟丢/滚偏，回 SURVEY 重来 */
                walking_direct_command(0, 0, 0);
                ESP_LOGW(TAG, "靠近时丢失 %s（最后 y=%.2f），重新等球",
                         s_color_name[s_current],
                         (double)s_seen[s_current].y_ratio);
                set_state(BALL_TASK_SURVEY, now_ms);
                break;
            }
        }

        /* 周期输出：朝球方位差速开进（球出画时按记忆方位盲走短窗） */
        const float off = (ball->detected ? ball->x_ratio
                                          : s_seen[s_current].x_ratio) - 0.5f;
        const int turn = (int)(off * BALL_PUSH_STEER_GAIN);
        walking_direct_command(BALL_APPROACH_SPEED_PCT, turn, 0);

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_APPROACH_TIMEOUT_MS) {
            walking_direct_command(0, 0, 0);
            ESP_LOGW(TAG, "靠近 %s 超时，重新等球", s_color_name[s_current]);
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;
    }

    case BALL_TASK_ALIGN: {
        /* 三点一线对准：横移消差（e=x球−x角标）+ 踢转居中。
         * 横移/踢转只在【新帧】重新决策（踢一脚-看一帧），帧间保持
         * 当前指令（踢转优先），保证低帧率下动作完整执行。 */
        const ball_blob_t *ball = pick_target_blob(balls);
        const bool ball_vis = ball->detected;
        const ball_blob_t *marker =
            pick_nearest_marker(balls,
                                ball_vis ? ball->x_ratio
                                         : s_seen[s_current].x_ratio,
                                NULL);

        if (new_frame) {
            if (ball_vis) {
                s_seen[s_current].x_ratio = ball->x_ratio;
                s_seen[s_current].y_ratio = ball->y_ratio;
                s_seen[s_current].area_ratio = ball->area_ratio;
                s_last_ball_seen_ms = now_ms;
            }

            if (ball_vis && marker != NULL) {
                /* -- 主路径：球与角标同帧可见 -- */
                const float e = ball->x_ratio - marker->x_ratio;
                const float pair_x = 0.5f * (ball->x_ratio + marker->x_ratio);
                const float pair_off = pair_x - 0.5f;
                /* 边缘保护按横移方向判定：横移会把画面整体推向移动的
                 * 反侧——e>0 右移时最左目标可能出左边缘，e<0 左移时
                 * 最右目标可能出右边缘。只有"要出去的一侧"贴边才禁
                 * 横移（旧版任一侧贴边即禁，会造成 twitch 死循环）。 */
                const float x_min = fminf(ball->x_ratio, marker->x_ratio);
                const float x_max = fmaxf(ball->x_ratio, marker->x_ratio);
                const bool near_edge =
                    (e > 0.0f) ? (x_min < BALL_ALIGN_EDGE_X)
                               : (x_max > 1.0f - BALL_ALIGN_EDGE_X);

                if (fabsf(e) <= BALL_ALIGN_DEADBAND &&
                    fabsf(pair_off) <= BALL_ALIGN_CENTER_BAND) {
                    /* 三点共线且目标对居中：撞球 */
                    walking_direct_command(0, 0, 0);
                    ESP_LOGI(TAG, "%s 三点一线对准完成（e=%.2f x=%.2f），撞球",
                             s_color_name[s_current], (double)e,
                             (double)pair_x);
                    set_state(BALL_TASK_PUSH, now_ms);
                    break;
                }
                if (fabsf(e) > BALL_ALIGN_DEADBAND && !near_edge) {
                    /* 横移消差：e>0（球在角标右）→ 连线在车右 → 右移。
                     * 移一脚-看一帧：只走一小段，停住等帧再评估；
                     * walking 约定 strafe>0=左移，方向现场可翻 SIGN。 */
                    s_align_strafe = 0;
                    s_kick_until_ms = 0;
                    schedule_strafe(e, now_ms);
                } else {
                    /* 已共线（或横移会把目标推出画面）：踢转把目标对
                     * 整体带向画面中心——旋转不改变 e，共线保持；等
                     * 贴边目标离开边缘后，横移自然解锁 */
                    s_align_strafe = 0;
                    schedule_kick(pair_off, now_ms);
                }
            } else if (ball_vis) {
                /* -- 兜底：角标不可见（未摆/未识别）——退化为踢转球
                 * 居中后直推，方向靠推球时的球跟踪与超时兜底 -- */
                s_align_strafe = 0;
                const float off = ball->x_ratio - 0.5f;
                if (fabsf(off) <= BALL_ALIGN_CENTER_BAND) {
                    walking_direct_command(0, 0, 0);
                    ESP_LOGI(TAG, "无角标可见，%s 已居中，直推",
                             s_color_name[s_current]);
                    set_state(BALL_TASK_PUSH, now_ms);
                    break;
                }
                schedule_kick(off, now_ms);
            } else {
                /* 本帧没球：停稳等下一帧，不盲动 */
                s_align_strafe = 0;
                s_kick_until_ms = 0;
                s_strafe_until_ms = 0;
            }
        }

        /* 周期输出：踢转 > 横移脉冲 > 停（脉冲结束到下一帧之间停住看帧） */
        const bool kicking = (s_kick_until_ms != 0) &&
                             (now_ms < s_kick_until_ms);
        const bool strafing = (s_strafe_until_ms != 0) &&
                              (now_ms < s_strafe_until_ms) && !kicking;
        if (kicking) {
            walking_direct_command(0, s_kick_dir * BALL_ALIGN_KICK_TURN_PCT, 0);
        } else if (strafing) {
            walking_direct_command(0, 0, s_align_strafe);
        } else {
            walking_direct_command(0, 0, 0);
        }

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_ALIGN_TIMEOUT_MS) {
            if (s_seen[s_current].y_ratio >= BALL_APPROACH_Y_ARRIVE_LOST) {
                /* 球已在车头近处而对准久久不成（如角标始终不可见）：
                 * 直接直推兜底，避免对准↔等球乒乓 */
                ESP_LOGW(TAG, "对准 %s 超时但球已近，直推兜底",
                         s_color_name[s_current]);
                set_state(BALL_TASK_PUSH, now_ms);
            } else {
                ESP_LOGW(TAG, "对准 %s 超时，重新等球",
                         s_color_name[s_current]);
                set_state(BALL_TASK_SURVEY, now_ms);
            }
        }

        /* 无球快速失败：对准中球不可见（如球在车头底下/滚偏出画）
         * 不干等满 12s，2s 即回 SURVEY（那里有蠕进/倒车重捕获兜底） */
        if (!ball_vis &&
            since_ms(now_ms, s_last_ball_seen_ms) >= BALL_ALIGN_LOST_WAIT_MS) {
            walking_direct_command(0, 0, 0);
            ESP_LOGW(TAG, "对准中 %s 不可见超过 %u ms，回等球",
                     s_color_name[s_current], (unsigned)BALL_ALIGN_LOST_WAIT_MS);
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;
    }

    case BALL_TASK_PUSH: {
        /* -- 新帧观测：球 / 最近角标 / 到位候选 -- */
        const ball_blob_t *ball = pick_target_blob(balls);
        const ball_blob_t *best_marker = NULL;
        float marker_dist = 1e9f;
        bool ball_vis = false;

        if (new_frame) {
            ball_vis = ball->detected;
            if (ball_vis) {
                s_seen[s_current].x_ratio = ball->x_ratio;
                s_seen[s_current].y_ratio = ball->y_ratio;
                s_seen[s_current].area_ratio = ball->area_ratio;
                s_last_ball_seen_ms = now_ms;
            }
            /* 选角标参考用最近一次球方位，目标不随球可见性跳变 */
            best_marker = pick_nearest_marker(balls, s_seen[s_current].x_ratio,
                                              &marker_dist);

            /* 到位判定：球与角标同帧可见且水平 x、纵向 y（≈地面深度）
             * 都贴近。纵向条件必不可少——三点一线推进时两者水平总是
             * 重合，只看 x 会在离角标还很远时就误判到位。 */
            if (ball_vis && best_marker != NULL) {
                const float dy = fabsf(ball->y_ratio - best_marker->y_ratio);
                if (marker_dist <= BALL_MARKER_ADJACENT_ERR_X &&
                    dy <= BALL_MARKER_ADJACENT_ERR_Y) {
                    if (s_push_done_pending_ms == 0) {
                        walking_direct_command(0, 0, 0);
                        s_push_done_pending_ms = now_ms;
                        ESP_LOGI(TAG, "%s 贴近角标（dx=%.2f dy=%.2f），静止确认",
                                 s_color_name[s_current], (double)marker_dist,
                                 (double)dy);
                    } else {
                        /* 第二帧（车已停）仍满足：该球完成，球被撞向角标 */
                        walking_direct_command(0, 0, 0);
                        s_pending[s_current] = false;
                        ESP_LOGI(TAG, "%s 已推到角标，%s",
                                 s_color_name[s_current],
                                 (s_pending[TARGET_RED] || s_pending[TARGET_BLUE])
                                     ? "退回推下一球" : "双球完成");
                        after_ball_finished(now_ms);
                        break;
                    }
                } else {
                    s_push_done_pending_ms = 0;
                }
            } else {
                s_push_done_pending_ms = 0;   /* 任一目标消失即取消待确认 */
            }
        }

        if (s_push_done_pending_ms != 0) {
            /* 静止等确认帧；太久无新帧（摄像头卡顿）则放弃这次确认继续推 */
            walking_direct_command(0, 0, 0);
            if (since_ms(now_ms, s_push_done_pending_ms) >
                BALL_TASK_DONE_CONFIRM_WAIT_MS) {
                ESP_LOGW(TAG, "到位确认超时（无新帧），继续推");
                s_push_done_pending_ms = 0;
                s_push_launch_start_ms = now_ms;   /* 重起步：再来一次强推缓降 */
            }
            break;
        }

        /* -- 运动输出（每周期）：转向 + 横移微修 + 前进 -- */
        float off = 0.0f;          /* 转向目标相对画面中心的偏差 */
        float strafe = 0.0f;       /* e 横移微修 */
        if (new_frame) {
            if (best_marker != NULL) {
                s_push_steer_target = best_marker->x_ratio;
            } else if (ball_vis) {
                s_push_steer_target = ball->x_ratio;
            }
        }
        if (ball_vis && best_marker != NULL) {
            const float e = ball->x_ratio - best_marker->x_ratio;
            if (fabsf(e) > BALL_ALIGN_DEADBAND) {
                /* 推进中三点一线被扰动：横移微修（力度小于对准档） */
                strafe = -BALL_ALIGN_STRAFE_SIGN *
                    (e > 0.0f ? BALL_PUSH_ALIGN_STRAFE_PCT
                              : -BALL_PUSH_ALIGN_STRAFE_PCT);
            }
        }
        off = s_push_steer_target - 0.5f;
        const int turn = (int)(off * BALL_PUSH_STEER_GAIN);

        if (since_ms(now_ms, s_last_ball_seen_ms) <= BALL_PUSH_BLIND_MS) {
            /* 推进速度：强推→缓降→巡航（起步防闷熄，见 push_speed_now）；
             * 球贴近车头滑出视野的短窗内按记忆方向盲走 */
            walking_direct_command(push_speed_now(now_ms), turn, strafe);
            s_push_drive_ms += BALL_CTRL_PERIOD_MS;
        } else {
            /* 盲走窗口耗尽仍看不到球：分两种情况，都不干等——
             * ① 球贴车头出画（最后位置高）：倒车一小段把球倒回视野，
             *    经 SURVEY 快速确认后重新衔接（直接对准会对着车底
             *    干等 12s，是此前"卡住"的主因）；
             * ② 球真丢了（最后位置低）：回 ALIGN 重新捕获。 */
            walking_direct_command(0, 0, 0);
            if (s_seen[s_current].y_ratio >= BALL_APPROACH_Y_ARRIVE_LOST) {
                ESP_LOGW(TAG, "%s 贴车头出画，倒车 %.0f ms 重新捕获",
                         s_color_name[s_current],
                         (double)BALL_REALIGN_BACKUP_MS);
                s_return_ms = BALL_REALIGN_BACKUP_MS;
                set_state(BALL_TASK_RETURN, now_ms);
            } else {
                ESP_LOGW(TAG, "%s 丢失超过 %u ms，停车重新对准",
                         s_color_name[s_current], (unsigned)BALL_PUSH_BLIND_MS);
                set_state(BALL_TASK_ALIGN, now_ms);
            }
            break;
        }

        /* 单球推球超时：视为完成该球的尝试，退回换下一球 */
        if (since_ms(now_ms, s_state_enter_ms) >= BALL_PUSH_TIMEOUT_MS) {
            walking_direct_command(0, 0, 0);
            s_pending[s_current] = false;
            ESP_LOGW(TAG, "%s 推球超时，放弃该球", s_color_name[s_current]);
            after_ball_finished(now_ms);
        }
        break;
    }

    case BALL_TASK_RETURN:
        /* 按推进累计时长开环盲退（近似原位；无里程计的折中） */
        walking_direct_command(-BALL_RETURN_SPEED_PCT, 0, 0);
        if (since_ms(now_ms, s_state_enter_ms) >= s_return_ms) {
            walking_direct_command(0, 0, 0);
            ESP_LOGI(TAG, "已退回原位附近，等待下一球");
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;

    case BALL_TASK_DONE:
    default:
        walking_direct_command(0, 0, 0);
        break;
    }
}
