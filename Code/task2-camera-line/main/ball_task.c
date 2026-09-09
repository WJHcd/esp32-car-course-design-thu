#include "ball_task.h"
#include "gimbal.h"
#include "walking.h"
#include "encoder.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * 推球状态机实现（v3 三点一线法）
 *
 * 视觉约定（来自 ball_detect，当前角标实物为黑色终点块）：
 *   - 红球/蓝球：ball_field 里各自的 detected + x_ratio + area_ratio；
 *   - 黑终点：marker[0..count-1]，按面积从大到小；
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
    "GIMBAL_UP", "SURVEY", "ALIGN", "PUSH", "RETURN", "DONE", "ABORT",
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
static uint32_t s_last_wait_log_ms;

/* 严格动作仲裁：任何短动作都必须完整经历
 * MOVE → BRAKE_SETTLE → WAIT_NEW_FRAME → IDLE，禁止被中途新帧续期。 */
typedef enum {
    ACTION_IDLE = 0,
    ACTION_MOVE,
    ACTION_BRAKE_SETTLE,
    ACTION_WAIT_NEW_FRAME,
} action_phase_t;

typedef struct {
    action_phase_t phase;
    float forward;
    float turn;
    float strafe;
    uint32_t until_ms;
    uint32_t wait_frame_seq;
    uint32_t hold_ms;      /* 动作后的刹停/停看时长（默认 SETTLE；
                              旋转搜索用 1.5s 停顿方便找球） */
    const char *reason;
    bool keep_drive;       /* 动作到期后不刹停，保持原指令滚到下一新帧
                              （推球段用：段间刹清动量导致车始终起不了步） */
} action_ctx_t;

static action_ctx_t s_action;
static uint8_t s_align_lost_frames;
static uint8_t s_align_recover_phase;    /* 丢球恢复：0=正常 1=后退中 2=自转已转 */
static bool s_red_pri_done;              /* 红球优先衔接已完成（每轮任务一次） */
static int s_red_pri_confirm;            /* 红球居中连续确认帧 */
static uint8_t s_red_pri_lost;           /* 红球连续不可见帧（闪断刹停用） */
static bool s_push_blind_step_used;
static bool s_camera_stale_logged;
static uint32_t s_push_done_pending_ms;  /* 0 = 无待确认；非 0 = 到位待静止确认 */
static uint32_t s_push_done_frame_seq;
static float s_done_prev_dist;           /* 到位确认首帧的球-角标距离 */
static uint32_t s_push_drive_ms;         /* PUSH 中前进指令累计时长（退回用） */
static float s_push_steer_target;        /* 最近一次生效转向目标（盲区续用） */
static uint32_t s_return_ms;             /* 本次退回计划时长 */
static int32_t s_push_pulses;            /* PUSH 期间双前轮里程脉冲（退回目标） */
static int32_t s_return_target_pulses;   /* 退回目标里程（0=按纯计时，重捕获用） */
static bool s_return_recapture;          /* true=贴车头重捕获短倒车（时长已定，
                                            * set_state 不得按推进时长重算） */

static inline uint32_t since_ms(uint32_t now_ms, uint32_t since_ms_)
{
    return now_ms - since_ms_;
}

static bool wait_log_due(uint32_t now_ms)
{
    if (since_ms(now_ms, s_last_wait_log_ms) < 500u) return false;
    s_last_wait_log_ms = now_ms;
    return true;
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

/* 角标锁定跟踪（2026-09-03 反馈文档 P1）：双角标同视野时每帧重选
 * "离球最近"会因检测抖动来回切换、e 跳变变号。进入 ALIGN 锁定首个
 * 可见角标的方位，此后按方位匹配同一目标；连续丢失若干帧才重选。 */
static bool s_marker_lock_valid;
static float s_marker_lock_x;
static float s_marker_lock_y;
static int s_marker_lock_lost;

/* 取当前推球目标的角标（带锁定）：ref_x = 最近一次球方位。
 * 返回 NULL = 本帧无可用角标（含"锁定目标暂丢、不换目标"）。 */
static const ball_blob_t *lock_or_pick_marker(const ball_field_result_t *b,
                                              float ref_x, float ref_y)
{
    const ball_blob_t *nearest = NULL;
    float nearest_dist = 1e9f;
    for (int i = 0; i < b->marker_count; i++) {
        const float dx = b->marker[i].x_ratio - ref_x;
        const float dy = b->marker[i].y_ratio - ref_y;
        const float d = dx * dx + dy * dy;
        if (d < nearest_dist) {
            nearest_dist = d;
            nearest = &b->marker[i];
        }
    }

    if (s_marker_lock_valid) {
        /* 按锁定方位匹配可见角标（框序号跨帧不稳，只能按位置跟踪） */
        const ball_blob_t *locked = NULL;
        float locked_dist = 1e9f;
        for (int i = 0; i < b->marker_count; i++) {
            const float dx = b->marker[i].x_ratio - s_marker_lock_x;
            const float dy = b->marker[i].y_ratio - s_marker_lock_y;
            const float d = dx * dx + dy * dy;
            if (d < locked_dist) {
                locked_dist = d;
                locked = &b->marker[i];
            }
        }
        if (locked != NULL &&
            locked_dist <= BALL_MARKER_MATCH_TOL * BALL_MARKER_MATCH_TOL) {
            s_marker_lock_x = locked->x_ratio;   /* 跟踪刷新方位 */
            s_marker_lock_y = locked->y_ratio;
            s_marker_lock_lost = 0;
            return locked;
        }
        /* 锁定目标本帧不可见：宁可不修正也不换目标，连续丢超限才放锁 */
        if (++s_marker_lock_lost <= BALL_MARKER_HOLD_LOST_FRAMES) {
            return NULL;
        }
        s_marker_lock_valid = false;
    }

    if (nearest != NULL) {
        s_marker_lock_valid = true;
        s_marker_lock_x = nearest->x_ratio;
        s_marker_lock_y = nearest->y_ratio;
        s_marker_lock_lost = 0;
    }
    return nearest;
}

/* PUSH/APPROACH 当前推进速度：强推 22%×350ms → 600ms 线性缓降至巡航。
 * 台阶式掉速会在死区上沿闷熄（车刚动起来就掉进 <9% 出力区）。
 * cruise 由调用方传入（PUSH/APPROACH 巡航档各自独立标定）。 */
static float push_speed_now(float cruise_pct)
{
    const uint32_t t = s_push_drive_ms;
    if (t >= BALL_PUSH_LAUNCH_MS + BALL_PUSH_RAMP_MS) {
        return cruise_pct;
    }
    if (t < BALL_PUSH_LAUNCH_MS) {
        return BALL_PUSH_LAUNCH_PCT;
    }
    const float k = (float)(t - BALL_PUSH_LAUNCH_MS) / (float)BALL_PUSH_RAMP_MS;
    return BALL_PUSH_LAUNCH_PCT -
           (BALL_PUSH_LAUNCH_PCT - cruise_pct) * k;
}

/* 低速差速限幅：A/C 双前轮中较慢的一侧 = speed - |turn|，跌进 9% 死区
 * 就会"一优异一劣、车体别住不走"（2026-09-03 反馈文档 P1）。球任务内
 * 保证 |turn| ≤ speed - BALL_MIN_EFFECTIVE_PCT，两侧都留出力余量。 */
static int clamp_turn_for_drive(float speed_pct, int turn)
{
    const int max_turn = (int)(speed_pct - BALL_MIN_EFFECTIVE_PCT);
    if (max_turn <= 0) return 0;
    if (turn > max_turn) return max_turn;
    if (turn < -max_turn) return -max_turn;
    return turn;
}

static uint32_t kick_duration_ms(float off)
{
    uint32_t kick = (uint32_t)(fabsf(off) * BALL_ALIGN_KICK_SCALE_MS);
    if (kick < BALL_ALIGN_KICK_MIN_MS) kick = BALL_ALIGN_KICK_MIN_MS;
    if (kick > BALL_ALIGN_KICK_MAX_MS) kick = BALL_ALIGN_KICK_MAX_MS;
    return kick;
}

static uint32_t strafe_duration_ms(float e)
{
    uint32_t pulse = (uint32_t)(fabsf(e) * BALL_ALIGN_STRAFE_SCALE_MS);
    if (pulse < BALL_ALIGN_STRAFE_MIN_MS) pulse = BALL_ALIGN_STRAFE_MIN_MS;
    if (pulse > BALL_ALIGN_STRAFE_MAX_MS) pulse = BALL_ALIGN_STRAFE_MAX_MS;
    return pulse;
}

static void reset_action(void)
{
    memset(&s_action, 0, sizeof(s_action));
    s_action.phase = ACTION_IDLE;
}

static void begin_action(float forward, float turn, float strafe,
                         uint32_t duration_ms, uint32_t now_ms,
                         const char *reason, bool keep_drive)
{
    s_action.phase = ACTION_MOVE;
    s_action.forward = forward;
    s_action.turn = turn;
    s_action.strafe = strafe;
    s_action.until_ms = now_ms + duration_ms;
    s_action.hold_ms = BALL_ACTION_SETTLE_MS;   /* 默认刹停 100ms，可按动作覆盖 */
    s_action.reason = reason;
    s_action.keep_drive = keep_drive;
    ESP_LOGI(TAG, "动作 %-14s F=%+.0f T=%+.0f S=%+.0f dur=%lu ms",
             reason, (double)forward, (double)turn, (double)strafe,
             (unsigned long)duration_ms);
    walking_direct_command(forward, turn, strafe);
}

/* 返回 true 表示动作链仍接管输出；false 表示已经拿到动作完成后的新帧，
 * 调用者可以基于该帧做下一次决策。 */
static bool service_action(const camera_line_result_t *cam, uint32_t now_ms)
{
    switch (s_action.phase) {
    case ACTION_MOVE:
        if (now_ms < s_action.until_ms) {
            walking_direct_command(s_action.forward, s_action.turn,
                                   s_action.strafe);
        } else if (s_action.keep_drive) {
            /* 推球段：到期不刹停，保持原指令滚入等帧窗口——段间刹清
             * 动量会让车每段都从静止对抗静摩擦，实测 20 段推不动。 */
            s_action.phase = ACTION_WAIT_NEW_FRAME;
            s_action.wait_frame_seq = cam->frame_seq;
            s_action.until_ms = now_ms + BALL_ACTION_WAIT_MAX_MS;
        } else {
            walking_direct_command(0, 0, 0);
            /* 刹停/停看窗口由 hold_ms 决定（默认 SETTLE 100ms；
             * 旋转搜索覆盖为 1.5s 停顿，方便小车找球）。 */
            s_action.phase = ACTION_BRAKE_SETTLE;
            s_action.until_ms = now_ms + s_action.hold_ms;
        }
        return true;

    case ACTION_BRAKE_SETTLE:
        walking_direct_command(0, 0, 0);
        if (now_ms >= s_action.until_ms) {
            s_action.phase = ACTION_WAIT_NEW_FRAME;
            s_action.wait_frame_seq = cam->frame_seq;
            s_action.until_ms = now_ms + BALL_ACTION_WAIT_MAX_MS;
        }
        return true;

    case ACTION_WAIT_NEW_FRAME:
        /* keep_drive 期间保持原指令滚动；超时（相机卡死/丢帧）强制
         * 交还决策权——PUSH 主流程会因无新帧而刹停。 */
        if (s_action.keep_drive) {
            walking_direct_command(s_action.forward, s_action.turn,
                                   s_action.strafe);
        } else {
            walking_direct_command(0, 0, 0);
        }
        if (cam->frame_seq != s_action.wait_frame_seq) {
            reset_action();
            return false;
        }
        if (now_ms >= s_action.until_ms) {
            ESP_LOGW(TAG, "动作 %s 等新帧超时 %s", s_action.reason,
                     s_action.keep_drive ? "强制刹停" : "放弃等待");
            reset_action();
            return false;
        }
        return true;

    case ACTION_IDLE:
    default:
        return false;
    }
}

static void set_state(ball_task_state_t st, uint32_t now_ms)
{
    s_state = st;
    s_state_enter_ms = now_ms;
    reset_action();
    /* 云台全程固定在扫视角（v3 起不再远近切换；重复设置无害） */
    gimbal_set_tilt(BALL_TASK_TILT_SURVEY_DEG);

    if (st == BALL_TASK_PUSH) {
        s_push_done_pending_ms = 0;
        s_push_drive_ms = 0;
        s_push_steer_target = 0.5f;
        s_push_blind_step_used = false;
        s_push_done_frame_seq = 0;
        encoder_odom_reset();              /* 推进里程从撞球起点起算 */
    } else if (st == BALL_TASK_RETURN) {
        /* 结算推进里程并清零重新计（退回段里程）。里程只含 PUSH 段：
         * 退回到的是"对准完成、开始撞球"的位置。 */
        s_push_pulses = (encoder_odom_get(MOTOR_A) +
                         encoder_odom_get(MOTOR_C)) / 2;
        encoder_odom_reset();
        if (s_return_recapture) {
            /* 贴车头重捕获短倒车：时长已由调用方给定（400ms），
             * 不得按推进时长重算（2026-09-03 反馈文档 P2）；按纯计时 */
            s_return_recapture = false;
            s_return_target_pulses = 0;
        } else {
            /* 推完一球退回原位：里程目标 = 推进脉冲 × 系数（编码器实测，
             * 死区/空转自动补偿）；计时保留为上限兜底 */
            uint32_t need = (uint32_t)((float)s_push_drive_ms * BALL_RETURN_TIME_SCALE);
            if (need < BALL_ALIGN_KICK_MIN_MS) need = BALL_ALIGN_KICK_MIN_MS;
            if (need > BALL_RETURN_MAX_MS) need = BALL_RETURN_MAX_MS;
            s_return_ms = need;
            s_return_target_pulses =
                (int32_t)((float)s_push_pulses * BALL_RETURN_PULSE_SCALE);
        }
        ESP_LOGI(TAG, "倒车 %lu ms（%s），里程目标 %ld 脉冲",
                 (unsigned long)s_return_ms,
                 s_return_ms <= BALL_REALIGN_BACKUP_MS ? "重捕获" : "退回原位",
                 (long)s_return_target_pulses);
    } else if (st == BALL_TASK_ALIGN) {
        s_align_lost_frames = 0;
        s_align_recover_phase = 0;   /* 全新对准：退出丢球恢复流程 */
        /* 重新锁定角标：上一球的目标关系不再适用 */
        s_marker_lock_valid = false;
        s_marker_lock_lost = 0;
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
    s_last_wait_log_ms = 0;
    s_camera_stale_logged = false;
    s_red_pri_done = false;              /* 新一轮任务：红球优先衔接重新生效 */
    s_red_pri_confirm = 0;
    reset_action();
    memset(s_seen, 0, sizeof(s_seen));
    s_red_pri_done = false;              /* 新一轮任务：红球优先衔接重新生效 */
    s_red_pri_confirm = 0;
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
    s_last_wait_log_ms = 0;
    s_camera_stale_logged = false;
    reset_action();
    memset(s_seen, 0, sizeof(s_seen));
    s_pending[TARGET_RED] = false;
    s_pending[TARGET_BLUE] = false;
}

bool ball_task_is_done(void)
{
    return s_state == BALL_TASK_DONE || s_state == BALL_TASK_ABORT;
}

bool ball_task_succeeded(void)
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

/* 选当前该推的球：只在本轮连续确认足够帧的待推球里选，避免 SURVEY
 * 超时时把一次偶发色块当成球。两个都确认时取面积大的（更近）。 */
static bool choose_next_target(void)
{
    const bool red_avail = s_pending[TARGET_RED] &&
                           s_seen[TARGET_RED].confirm >= BALL_SURVEY_CONFIRM;
    const bool blue_avail = s_pending[TARGET_BLUE] &&
                            s_seen[TARGET_BLUE].confirm >= BALL_SURVEY_CONFIRM;
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

/* SURVEY 只接受连续、位置相近的不同帧。一次偶发误检不能隔很多帧后
 * 与另一次误检拼成“确认成功”。 */
static void update_survey_sighting(target_color_t target,
                                   const ball_blob_t *blob)
{
    ball_sighting_t *seen = &s_seen[target];
    if (!blob->detected) {
        seen->confirm = 0;
        return;
    }
    const bool continuous = seen->confirm > 0 &&
        fabsf(blob->x_ratio - seen->x_ratio) <= BALL_SURVEY_TRACK_JUMP_MAX &&
        fabsf(blob->y_ratio - seen->y_ratio) <= BALL_SURVEY_TRACK_JUMP_MAX;
    seen->confirm = continuous ? seen->confirm + 1 : 1;
    seen->x_ratio = blob->x_ratio;
    seen->y_ratio = blob->y_ratio;
    seen->area_ratio = blob->area_ratio;
}

/* 全任务兜底超时 */
static bool task_timed_out(uint32_t now_ms)
{
    return since_ms(now_ms, s_total_start_ms) >= BALL_TASK_TOTAL_MS;
}

/* 仅在视觉连续两帧确认该球到位后调用。 */
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
    /* 喂里程计采样（内部 200ms 限速，20ms 周期调用无害；
     * 未初始化编码器时为空转，RETURN 自动退回纯计时） */
    encoder_update();

    if (cam->frame_seq == 0 ||
        since_ms(now_ms, cam->frame_ms) > BALL_CAMERA_STALE_MS) {
        walking_direct_command(0, 0, 0);
        reset_action();
        if (!s_camera_stale_logged) {
            s_camera_stale_logged = true;
            ESP_LOGE(TAG, "视觉帧超过 %u ms 未更新，安全停车等待恢复",
                     (unsigned)BALL_CAMERA_STALE_MS);
        }
        return;
    }
    if (s_camera_stale_logged) {
        ESP_LOGI(TAG, "视觉帧恢复（frame=%lu）",
                 (unsigned long)cam->frame_seq);
        s_camera_stale_logged = false;
    }

    if (s_state != BALL_TASK_DONE && s_state != BALL_TASK_ABORT &&
        task_timed_out(now_ms)) {
        walking_direct_command(0, 0, 0);
        ESP_LOGW(TAG, "找球任务总超时，安全中止（不标记成功）");
        set_state(BALL_TASK_ABORT, now_ms);
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
        /* 安全测试默认完全静止。自主蠕进必须显式打开宏，不能在没有
         * 可靠目标时悄悄开始移动。 */
        const bool rotating = false;
        const bool creeping = BALL_SURVEY_CREEP_ENABLE &&
            since_ms(now_ms, s_state_enter_ms) >= BALL_SURVEY_CREEP_DELAY_MS;
#endif
        /* 红球优先衔接（2026-09-04 用户指定）：避障巡线后车身朝向偏右，
         * 若不修正会先撞上右边的蓝球。此段在红球仍待推时生效：原地
         * 逆时针旋转寻找红球 → 找到后转到画面中央 → 直接对准推红球。
         * 红球推完（pending 清除）后此段自动失效，正常流程推蓝球；
         * 20s 找不到红球则放弃优先，恢复正常等球流程。 */
        if (s_pending[TARGET_RED] && !s_red_pri_done) {
            if (!new_frame) {
                break;
            }
            if (since_ms(now_ms, s_state_enter_ms) >= BALL_SURVEY_TIMEOUT_MS) {
                ESP_LOGW(TAG, "红球优先衔接超时，放弃优先恢复正常流程");
                s_red_pri_done = true;
                break;
            }
            if (balls->red_ball.detected) {
                const float off = balls->red_ball.x_ratio - 0.5f;
                s_align_lost_frames = 0;
                s_red_pri_lost = 0;
                if (fabsf(off) > 0.15f) {
                    /* 红球偏哪边就往哪边转，把它带到画面中央附近
                     * （全速一步 ~17°，死区放宽到 15% 防帧间过冲） */
                    walking_direct_command(0, (off > 0.0f ? 1 : -1) *
                                              BALL_SURVEY_TURN_PCT, 0);
                    s_red_pri_confirm = 0;
                    break;
                }
                /* |off| ≤ 0.15：刹停防过冲，连续 2 帧确认即视为居中 */
                walking_direct_command(0, 0, 0);
                ++s_red_pri_confirm;
                if (s_red_pri_confirm < 2) {
                    break;                 /* 居中后再确认一帧，防抖 */
                }
                /* 红球已近似居中：锁定红球为目标，直接进入对准 */
                s_current = TARGET_RED;
                s_seen[TARGET_RED].confirm = BALL_SURVEY_CONFIRM;
                s_seen[TARGET_RED].x_ratio = balls->red_ball.x_ratio;
                s_seen[TARGET_RED].y_ratio = balls->red_ball.y_ratio;
                s_seen[TARGET_RED].area_ratio = balls->red_ball.area_ratio;
                s_red_pri_done = true;
                ESP_LOGI(TAG, "红球优先衔接完成（已居中 x=%+.2f），直接对准推红球",
                         (double)balls->red_ball.x_ratio);
                set_state(BALL_TASK_ALIGN, now_ms);
                break;
            }
            /* 本帧无红球：可能是检测闪断（上一帧已近居中）——先刹停
             * 等一帧再判断；连续 3 帧确认真没有才恢复逆时针旋转寻找 */
            ++s_red_pri_lost;
            if (s_red_pri_lost < 3) {
                walking_direct_command(0, 0, 0);
                break;
            }
            s_red_pri_confirm = 0;
            walking_direct_command(0, -BALL_SURVEY_TURN_PCT, 0);
            break;
        }
        if (new_frame && !rotating) {
            update_survey_sighting(TARGET_RED, &balls->red_ball);
            update_survey_sighting(TARGET_BLUE, &balls->blue_ball);

            const bool red_ok = !s_pending[TARGET_RED] ||
                                s_seen[TARGET_RED].confirm >= BALL_SURVEY_CONFIRM;
            const bool blue_ok = !s_pending[TARGET_BLUE] ||
                                 s_seen[TARGET_BLUE].confirm >= BALL_SURVEY_CONFIRM;

            if (red_ok && blue_ok) {
                /* 双球都确认：按面积选近的先推 */
                walking_direct_command(0, 0, 0);
                if (choose_next_target()) {
                    set_state(BALL_TASK_ALIGN, now_ms);
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
                set_state(BALL_TASK_ALIGN, now_ms);
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
                set_state(BALL_TASK_ALIGN, now_ms);
            } else {
                ESP_LOGW(TAG, "等待超时且未发现任何球，安全中止");
                set_state(BALL_TASK_ABORT, now_ms);
            }
        }
        break;
    }

    case BALL_TASK_ALIGN: {
        /* 参考 ball_pushing 的闭环节拍：短动作结束后先刹停，再等一张
         * 真正的新画面。动作期间的新帧不会续期，也不会叠加第二条指令。 */
        const ball_blob_t *ball_pre = pick_target_blob(balls);
        /* 后退找球中任何一帧重新见球：立即刹停，本帧即按正常对准流程
         * 继续推球（用户定稿 2026-09-04）。 */
        if (s_align_recover_phase != 0 && ball_pre->detected) {
            reset_action();
            s_align_recover_phase = 0;
            s_align_lost_frames = 0;
            ESP_LOGI(TAG, "后退找球中找回%s（x=%+.2f），恢复对准推球",
                     s_color_name[s_current], (double)ball_pre->x_ratio);
        } else if (service_action(cam, now_ms)) {
            break;
        }
        walking_direct_command(0, 0, 0);

        if (since_ms(now_ms, s_state_enter_ms) >= BALL_ALIGN_TIMEOUT_MS) {
            ESP_LOGW(TAG, "对准 %s 超时，停车重新观察",
                     s_color_name[s_current]);
            set_state(BALL_TASK_SURVEY, now_ms);
            break;
        }
        if (!new_frame) {
            break;
        }

        const ball_blob_t *ball = pick_target_blob(balls);
        if (!ball->detected) {
            if (s_align_recover_phase == 1) {
                /* 后退 1.2s 完成仍无球：顺时针自转一次（幅度稍大 400ms），
                 * 转完回找球状态（SURVEY）。 */
                s_align_recover_phase = 2;
                begin_action(0, BALL_SURVEY_DIR * BALL_SURVEY_TURN_PCT, 0,
                             BALL_ALIGN_SEARCH_TURN_MS, now_ms,
                             "ALIGN_CW_SEARCH", false);
                break;
            }
            if (s_align_recover_phase == 2) {
                /* 自转一次后仍无球：回找球状态（SURVEY）等待球/重新确认；
                 * ALIGN 20s 总超时仍在兜底。 */
                ESP_LOGW(TAG, "自转一次后仍未见%s，回找球状态",
                         s_color_name[s_current]);
                set_state(BALL_TASK_SURVEY, now_ms);
                break;
            }
            if (s_align_lost_frames < UINT8_MAX) {
                s_align_lost_frames++;
            }
            if (s_align_lost_frames < BALL_LOST_CONFIRM_FRAMES) {
                break;
            }
            /* 用户定稿（2026-09-04）：丢球 → 连续后退 1.2s（后退中任何
             * 一帧重新见球，立即刹停恢复对准推球）；退完仍无球 → 顺时针
             * 自转一次。 */
            s_align_recover_phase = 1;
            begin_action(-BALL_ALIGN_BWD_PCT, 0, 0,
                         BALL_ALIGN_BACKUP_MS, now_ms,
                         "ALIGN_BACKUP", false);
            break;
        }

        s_align_lost_frames = 0;
        s_seen[s_current].x_ratio = ball->x_ratio;
        s_seen[s_current].y_ratio = ball->y_ratio;
        s_seen[s_current].area_ratio = ball->area_ratio;

        const ball_blob_t *marker =
            lock_or_pick_marker(balls, ball->x_ratio, ball->y_ratio);
        if (marker == NULL) {
            /* 首阶段必须同时看稳球和黑色终点才允许移动。画面能看到但
             * 算法没框时保持静止，避免盲目前进把误检放大成满场乱跑。 */
            if (wait_log_due(now_ms)) {
                ESP_LOGI(TAG, "已见%s但本帧无稳定黑色终点，保持停车",
                         s_color_name[s_current]);
            }
            break;
        }

        const float e = ball->x_ratio - marker->x_ratio;
        const float pair_x = 0.5f * (ball->x_ratio + marker->x_ratio);
        const float pair_off = pair_x - 0.5f;
        const float x_min = fminf(ball->x_ratio, marker->x_ratio);
        const float x_max = fmaxf(ball->x_ratio, marker->x_ratio);
        const bool near_edge =
            (e > 0.0f) ? (x_min < BALL_ALIGN_EDGE_X)
                       : (x_max > 1.0f - BALL_ALIGN_EDGE_X);

        if (fabsf(e) <= BALL_ALIGN_DEADBAND &&
            fabsf(pair_off) <= BALL_ALIGN_CENTER_BAND) {
            ESP_LOGI(TAG, "%s 与黑色终点对准（e=%.2f x=%.2f），进入分段推球",
                     s_color_name[s_current], (double)e, (double)pair_x);
            set_state(BALL_TASK_PUSH, now_ms);
        } else if (fabsf(e) > BALL_ALIGN_DEADBAND && !near_edge) {
            const float strafe = -BALL_ALIGN_STRAFE_SIGN *
                (e > 0.0f ? BALL_ALIGN_STRAFE_PCT : -BALL_ALIGN_STRAFE_PCT);
            begin_action(0, 0, strafe, strafe_duration_ms(e), now_ms,
                         "ALIGN_STRAFE", false);
        } else {
            /* 两目标分居画面两侧、pair_off 恰好接近 0 时，旧写法会因
             * “0 不大于 0”固定选择负方向，形成明显左右不对称。此时
             * 按即将出界的一侧选方向：左侧受威胁则把画面带向右，
             * 右侧受威胁则把画面带向左。 */
            float turn_off = pair_off;
            if (near_edge && fabsf(turn_off) <= BALL_ALIGN_CENTER_BAND) {
                turn_off = (e > 0.0f) ? -BALL_ALIGN_CENTER_BAND
                                      : BALL_ALIGN_CENTER_BAND;
            }
            const float turn = (turn_off > 0.0f)
                ? BALL_ALIGN_KICK_TURN_PCT : -BALL_ALIGN_KICK_TURN_PCT;
            begin_action(0, turn, 0, kick_duration_ms(turn_off), now_ms,
                         "ALIGN_TURN", false);
        }
        break;
    }

    case BALL_TASK_PUSH: {
        /* 推球同样采用“动作→刹停→新帧”的离散闭环。 */
        if (service_action(cam, now_ms)) {
            break;
        }
        walking_direct_command(0, 0, 0);

        /* 限距/超时只表示失败，绝不能清 pending 冒充已推到终点。 */
        if (s_push_drive_ms >= BALL_PUSH_MAX_DRIVE_MS) {
            ESP_LOGE(TAG, "%s 推进累计达到 %u ms，安全中止（未确认到位）",
                     s_color_name[s_current], (unsigned)BALL_PUSH_MAX_DRIVE_MS);
            set_state(BALL_TASK_ABORT, now_ms);
            break;
        }
        if (since_ms(now_ms, s_state_enter_ms) >= BALL_PUSH_TIMEOUT_MS) {
            ESP_LOGE(TAG, "%s 推球超时，安全中止（未确认到位）",
                     s_color_name[s_current]);
            set_state(BALL_TASK_ABORT, now_ms);
            break;
        }
        if (!new_frame) {
            break;
        }

        const ball_blob_t *ball = pick_target_blob(balls);
        const bool ball_vis = ball->detected;
        const ball_blob_t *marker = NULL;
        float adjacent_dist = 1e9f;
        float adjacent_dx = 1e9f;
        float adjacent_dy = 1e9f;
        bool adjacent = false;

        if (ball_vis) {
            s_seen[s_current].x_ratio = ball->x_ratio;
            s_seen[s_current].y_ratio = ball->y_ratio;
            s_seen[s_current].area_ratio = ball->area_ratio;
            s_align_lost_frames = 0;
            s_push_blind_step_used = false;
            marker = lock_or_pick_marker(balls, ball->x_ratio, ball->y_ratio);

            /* 到位使用二维最近邻，避免另一块黑色终点在水平位置接近时
             * 被误配。 */
            for (int i = 0; i < balls->marker_count; i++) {
                const float dx = fabsf(balls->marker[i].x_ratio - ball->x_ratio);
                const float dy = fabsf(balls->marker[i].y_ratio - ball->y_ratio);
                const float dist = sqrtf(dx * dx + dy * dy);
                if (dist < adjacent_dist) {
                    adjacent_dist = dist;
                    adjacent_dx = dx;
                    adjacent_dy = dy;
                }
            }
            adjacent = adjacent_dx <= BALL_MARKER_ADJACENT_ERR_X &&
                       adjacent_dy <= BALL_MARKER_ADJACENT_ERR_Y;
        } else if (s_align_lost_frames < UINT8_MAX) {
            s_align_lost_frames++;
        }

        /* 必须是两个不同的新帧，且两帧间车辆停车，才确认到位。 */
        if (s_push_done_pending_ms != 0) {
            if (cam->frame_seq != s_push_done_frame_seq && adjacent &&
                adjacent_dist <= s_done_prev_dist + BALL_DONE_DIST_EPS) {
                s_pending[s_current] = false;
                ESP_LOGI(TAG, "%s 已连续两帧贴近黑色终点，%s",
                         s_color_name[s_current],
                         (s_pending[TARGET_RED] || s_pending[TARGET_BLUE])
                             ? "退回后处理下一球" : "双球任务成功");
                after_ball_finished(now_ms);
                break;
            }
            ESP_LOGW(TAG, "%s 到位复核未通过，取消本次候选",
                     s_color_name[s_current]);
            s_push_done_pending_ms = 0;
            s_push_done_frame_seq = 0;
        }
        if (adjacent && s_push_drive_ms >= BALL_DONE_MIN_DRIVE_MS) {
            s_push_done_pending_ms = now_ms;
            s_push_done_frame_seq = cam->frame_seq;
            s_done_prev_dist = adjacent_dist;
            ESP_LOGI(TAG, "%s 接近黑色终点（dx=%.2f dy=%.2f），停车等下一帧复核",
                     s_color_name[s_current], (double)adjacent_dx,
                     (double)adjacent_dy);
            break;
        }

        if (!ball_vis) {
            if (s_align_lost_frames < BALL_LOST_CONFIRM_FRAMES) {
                break;
            }
            if (!s_push_blind_step_used &&
                s_seen[s_current].y_ratio >= BALL_NEAR_Y_ARRIVE_LOST) {
                s_push_blind_step_used = true;
                s_push_drive_ms += BALL_PUSH_BLIND_STEP_MS;
                begin_action(BALL_PUSH_SPEED_PCT, 0, 0,
                             BALL_PUSH_BLIND_STEP_MS, now_ms,
                             "PUSH_LAST_STEP", true);
            } else if (s_seen[s_current].y_ratio >= BALL_NEAR_Y_ARRIVE_LOST) {
                ESP_LOGW(TAG, "%s 近车头连续丢失，短倒车重新捕获",
                         s_color_name[s_current]);
                s_return_recapture = true;
                s_return_ms = BALL_REALIGN_BACKUP_MS;
                set_state(BALL_TASK_RETURN, now_ms);
            } else {
                ESP_LOGW(TAG, "%s 连续 %u 帧丢失，停车重新对准",
                         s_color_name[s_current],
                         (unsigned)s_align_lost_frames);
                set_state(BALL_TASK_ALIGN, now_ms);
            }
            break;
        }

        if (marker == NULL) {
            /* 已经进入 PUSH 也不能因为黑色终点短暂消失就沿旧方向连续
             * 推。保持停车，下一帧重试；持续看不到最终会明确超时中止。 */
            if (wait_log_due(now_ms)) {
                ESP_LOGI(TAG, "推球中本帧无稳定黑色终点，保持停车");
            }
            break;
        }

        const float e = ball->x_ratio - marker->x_ratio;
        if (fabsf(e) > BALL_ALIGN_DEADBAND) {
            const float strafe = -BALL_ALIGN_STRAFE_SIGN *
                (e > 0.0f ? BALL_PUSH_ALIGN_STRAFE_PCT
                          : -BALL_PUSH_ALIGN_STRAFE_PCT);
            begin_action(0, 0, strafe, strafe_duration_ms(e), now_ms,
                         "PUSH_REALIGN", false);
            break;
        }
        s_push_steer_target = marker->x_ratio;

        const float off = s_push_steer_target - 0.5f;
        if (fabsf(off) > BALL_ALIGN_CENTER_BAND) {
            const float turn = off > 0.0f ? BALL_ALIGN_KICK_TURN_PCT
                                         : -BALL_ALIGN_KICK_TURN_PCT;
            begin_action(0, turn, 0, kick_duration_ms(off), now_ms,
                         "PUSH_CENTER", false);
            break;
        }

        const float cruise = push_speed_now(BALL_PUSH_SPEED_PCT);
        const int turn = clamp_turn_for_drive(
            cruise, (int)((ball->x_ratio - 0.5f) * BALL_PUSH_STEER_GAIN));
        s_push_drive_ms += BALL_PUSH_STEP_MS;
        begin_action(cruise, turn, 0, BALL_PUSH_STEP_MS, now_ms,
                     "PUSH_STEP", true);
        break;
    }

    case BALL_TASK_RETURN: {
        /* 里程计退回：倒到"双前轮累计脉冲 ≥ 推进里程 × 系数"为止
         * （死区/卡死不转时不计里程，自动多倒补偿）；计时为上限兜底，
         * 重捕获短倒车 s_return_target_pulses=0，退化为纯计时。 */
        walking_direct_command(-BALL_RETURN_SPEED_PCT, 0, 0);
        const int32_t back_pulses = (encoder_odom_get(MOTOR_A) +
                                     encoder_odom_get(MOTOR_C)) / 2;
        const bool dist_done = (s_return_target_pulses > 0) &&
                               (back_pulses >= s_return_target_pulses);
        if (dist_done || since_ms(now_ms, s_state_enter_ms) >= s_return_ms) {
            walking_direct_command(0, 0, 0);
            ESP_LOGI(TAG, "退回结束（%ld/%ld 脉冲，%s），等待下一球",
                     (long)back_pulses, (long)s_return_target_pulses,
                     dist_done ? "里程到位" : "计时上限");
            set_state(BALL_TASK_SURVEY, now_ms);
        }
        break;
    }

    case BALL_TASK_DONE:
    case BALL_TASK_ABORT:
    default:
        walking_direct_command(0, 0, 0);
        break;
    }
}
