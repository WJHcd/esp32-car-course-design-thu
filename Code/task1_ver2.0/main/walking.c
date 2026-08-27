#include "walking.h"
#include "motor.h"
#include "ultrasonic.h"
#include <math.h>
#include <stdlib.h>

/* ------------------------ 现场标定区 ------------------------
 * 轮子命名按参考底盘：L=左轮、R=右轮、B=后轮。
 * 实车已确认：A=左轮、C=右轮、B=后轮；三路电机逆时针均为正。
 * 如果后续重新接线或改装轮位，只改下面两组映射，不改控制逻辑。
 */
#define WHEEL_TO_MOTOR_L   MOTOR_A
#define WHEEL_TO_MOTOR_R   MOTOR_C
#define WHEEL_TO_MOTOR_B   MOTOR_B

static const int s_motor_sign[MOTOR_NUM] = { +1, +1, +1 };

/*
 * 同一 PWM 下 C 轮实测略快于 A 轮。这里是电机输出标定，不改变运动学方向；
 * 后续如仍有直行偏差，只在 0.85~1.00 间微调 C 的系数。
 */
static const float s_motor_gain[MOTOR_NUM] = { 1.00f, 1.00f, 0.90f };

/*
 * 正前进（车头方向）的现场标定：中心线实测旧组合 A 正、C 反会后退，
 * 因此仅翻转前进分量。保留 turn 分量不变，避免改变已经单独标定的转向方向。
 * forward > 0 时：A 反转、C 正转、B 停止。
 */
#define FORWARD_SIGN_L     (-1)
#define FORWARD_SIGN_R     (+1)

/*
 * 实车循迹验证：黑线在右侧时旧 B 转向分量会使车继续向右偏。
 * A/C 的转向分量保持不变，仅翻转后轮 B 的转向分量。
 */
#define TURN_SIGN_L        (+1)
#define TURN_SIGN_R        (+1)
#define TURN_SIGN_B        (-1)

/* 后轮只承担转向；0.65 倍介于原 0.60 与死区补偿后的等效 0.70 之间的实测折中。 */
#define TURN_B_GAIN         0.65f

/* 若误差为正（黑线在车体右侧）时实车反而向左修正，把它改成 -1。 */
#define STEERING_SIGN       (+1)

/* ---- 平移（横移）标定：约定 strafe>0 = 向左平移 ----
 * 初始值按三轮全向对称猜测（L/R 同向、B 反向）；必须用 TASK1_MODE=5
 * 架空轮子实测：若 strafe>0 时车体实际向右，翻转 STRAFE_SIGN_L/R/B 符号。
 * 标定方式与 FORWARD_SIGN_* / TURN_SIGN_* 完全一致，不影响循迹。
 */
#define STRAFE_SIGN_L       (+1)
#define STRAFE_SIGN_R       (+1)
#define STRAFE_SIGN_B       (-1)
/*
 * 后轮平移增益：按 120° 三轮全向几何，纯平移时 B 轮转速约为 L/R 的 2 倍
 * （L:+v, R:+v, B:-2v）。实车若平移时车体带旋转，再在 1.5~2.5 间微调。
 */
#define STRAFE_B_GAIN       2.0f

/* ------------------------ 控制参数区 ------------------------ */
#define LINE_BASE_SPEED_PCT       12
#define LINE_MIN_SPEED_PCT         8
#define LINE_MAX_TURN_PCT         10
#define LINE_KP                    3.0f
#define LINE_KD                    0.5f  /* 少量微分阻尼，抑制直线来回扭动 */
#define LINE_TURN_SLEW_PCT         3  /* 每个 20ms 控制周期最大转向变化量 */
#define LINE_ERROR_FILTER_ALPHA  0.30f /* 数字红外的低通系数，越小越稳但越慢 */
#define LINE_CENTER_DEADBAND      0.45f /* 小误差不转向，避免直线来回扭动 */
#define LINE_SEARCH_SIDE_MIN_ERROR 0.25f /* 参与方向投票所需的最小原始偏差 */
#define LINE_SIDE_ACCUM_WINDOW     10  /* 方向投票窗口（拍数，20ms/拍 ≈ 200ms） */
#define LINE_SIDE_FLIP_MIN          2  /* 累计票达到该值才允许翻转找线方向 */
#define LINE_LOST_GRACE_MS        60u
#define LINE_LOST_FORWARD_PCT      7
#define LINE_SEARCH_TIMEOUT_MS     0u /* 0=只红外测试时持续找线，直到重新见线 */
#define LINE_SEARCH_TURN_PCT       12
#define LINE_CORNER_SPEED_PCT      7
#define LINE_CORNER_TURN_PCT      12
/*
 * 锐角弯专用找出口参数：
 * - 结束条件始终是确认重新见线，不是定时结束；
 * - 重试时间只用于判断首选方向很可能错误，届时仅反向尝试一次。
 */
#define LINE_CORNER_EXIT_CONFIRM_SAMPLES  3u
#define LINE_CORNER_RETRY_MS           1000u
/* 反向重试后仍找不到出口的总兜底时间：超时进故障停车，避免无限原地空转。 */
#define LINE_CORNER_TIMEOUT_MS         5000u

#define OBSTACLE_TRIGGER_CM       18u
#define OBSTACLE_CLEAR_CM         24u
#define OBSTACLE_CONFIRM_SAMPLES   2u
#define OBSTACLE_SAMPLE_GAP_MS    350u

/* ---- 硬编码平移避障时序（赛道简单，时长/速度实车再调）----
 * 约定：左/右平移时长 > 前进时长（初值 800ms / 500ms / 800ms）。
 * 调整时整体缩放即可，保持左/右 > 前进。 */
#define AVOID_STOP_MS            120u
#define AVOID_STRAFE_LEFT_MS     800u
#define AVOID_FORWARD_MS         500u
#define AVOID_STRAFE_RIGHT_MS    800u
#define AVOID_FIND_LINE_MS      1800u
#define AVOID_STRAFE_SPEED_PCT   12
#define AVOID_FORWARD_SPEED_PCT  15
#define AVOID_FIND_SPEED_PCT     10

/* 找线阶段小幅转向扫描的偏置方向（原 AVOID_TURN_SIGN 改名）；±1 按实车微调。 */
#define AVOID_SCAN_TURN_SIGN    (+1)

static const char *const s_state_name[] = {
    "RUN",
    "LINE_SEARCH",
    "CORNER_SEARCH",
    "AVOID_STOP",
    "AVOID_STRAFE_LEFT",
    "AVOID_FORWARD",
    "AVOID_STRAFE_RIGHT",
    "AVOID_FIND_LINE",
    "FAILSAFE_STOP",
};

static car_state_t s_state;
static uint32_t s_state_enter_ms;
static uint32_t s_line_lost_since_ms;
static uint32_t s_last_distance_sample_ms;
static uint8_t s_obstacle_samples;
static uint8_t s_line_reacquire_samples;
static uint8_t s_corner_exit_samples;
static bool s_line_lost_timing;
static bool s_last_line_was_corner;
static bool s_corner_retry_used;
static bool s_output_enabled;
static float s_last_error;
/* +1=黑线最后在右侧，应使用正 turn；-1=最后在左侧，应使用负 turn。 */
static int s_last_line_side;
/* 方向投票计数器：正=近期线在右侧占优，负=左侧占优。 */
static int s_line_side_accum;
static int s_corner_turn_side;
static float s_filtered_error;
static bool s_error_filter_valid;
static walking_command_t s_last_command;

static int clamp_int(int value, int low, int high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

static uint32_t elapsed_ms(uint32_t now_ms, uint32_t since_ms)
{
    return now_ms - since_ms;  /* unsigned subtraction also handles wraparound */
}

static void enter_state(car_state_t state, uint32_t now_ms)
{
    s_state = state;
    s_state_enter_ms = now_ms;
    s_line_reacquire_samples = 0;
}

/*
 * 当前底盘三轴混控（forward / turn / strafe）：
 *   纯前进（已实车校正）：L=-v, R=+v, B=0
 *   原地旋转：左右轮同向转向分量、后轮 B 反向分量（0.65 倍补偿）
 *   平移（strafe>0=左）：L/R 按 STRAFE_SIGN、后轮 B 反向（增益待标定）
 * 循迹只用到 forward/turn（strafe=0），输出与旧版完全一致，不影响循迹。
 * 三轮比例超出 100% 时等比缩放，避免逐轮截断改变运动方向。
 */
static void apply_body_command(int forward, int turn, int strafe)
{
    s_last_command.forward_percent = forward;
    s_last_command.turn_percent = turn;
    s_last_command.strafe_percent = strafe;

    float wheel[MOTOR_NUM] = {0};
    wheel[WHEEL_TO_MOTOR_L] = (float)(FORWARD_SIGN_L * forward + TURN_SIGN_L * turn + STRAFE_SIGN_L * strafe);
    wheel[WHEEL_TO_MOTOR_R] = (float)(FORWARD_SIGN_R * forward + TURN_SIGN_R * turn + STRAFE_SIGN_R * strafe);
    wheel[WHEEL_TO_MOTOR_B] = (float)(TURN_SIGN_B * turn) * TURN_B_GAIN
                            + (float)(STRAFE_SIGN_B * strafe) * STRAFE_B_GAIN;

    float max_abs = 100.0f;
    for (int i = 0; i < MOTOR_NUM; i++) {
        const float value_abs = fabsf(wheel[i]);
        if (value_abs > max_abs) {
            max_abs = value_abs;
        }
    }

    if (!s_output_enabled) {
        return;
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        const int percent = (int)lroundf(wheel[i] * s_motor_gain[i] * 100.0f / max_abs);
        motor_set_speed(i, percent * s_motor_sign[i]);
    }
}

static void apply_brake(void)
{
    s_last_command.forward_percent = 0;
    s_last_command.turn_percent = 0;
    s_last_command.strafe_percent = 0;
    if (s_output_enabled) {
        motor_stop_all();
    }
}

/* 仅测试/标定用：绕过状态机直接驱动三轴（见 walking.h）。 */
void walking_test_drive(int forward, int turn, int strafe)
{
    apply_body_command(forward, turn, strafe);
}

/* 避免数字红外从中心跳到外侧时，转向指令在一个控制周期内突变。 */
static int limit_line_turn_slew(int requested_turn)
{
    const int previous_turn = s_last_command.turn_percent;
    return clamp_int(requested_turn,
                     previous_turn - LINE_TURN_SLEW_PCT,
                     previous_turn + LINE_TURN_SLEW_PCT);
}

/*
 * 外侧探头参与命中即视为接近直角/锐角弯：a&&!c 或 d&&!b
 * （覆盖外侧单点 a/d，也覆盖外侧+内侧 a&&b / c&&d）。
 * 该判定只负责两件事：锁定找线方向、置弯道标志（供全白后进 CORNER_SEARCH），
 * 不直接决定控制动作；动作区分见 run_line_controller。
 */
static bool infrared_is_sharp_corner(const infrared_pattern_t *pat)
{
    return (pat->a && !pat->c) || (pat->d && !pat->b);
}

/*
 * 外侧单探头命中、相邻内侧未命中（a&&!b / d&&!c）：马上就要丢线，
 * 只有这种图样才用固定大转向的弯道动作；a&&b / c&&d 只是偏得多，走正常 PD。
 */
static bool infrared_is_corner_imminent(const infrared_pattern_t *pat)
{
    return (pat->a && !pat->b) || (pat->d && !pat->c);
}

/*
 * 锐角弯出口只接受与转弯方向同侧的内侧探头：左转时线必须从左内(b)进入，
 * 右转时从右内(c)进入。对侧探头先见线说明转过头或扫到了身后的来路线，
 * 不能算出口；仅凭外侧探头的短暂命中也不算出口。
 */
static bool infrared_is_corner_exit_candidate(const infrared_pattern_t *pat,
                                              int turn_side)
{
    return (turn_side < 0) ? (pat->b != 0) : (pat->c != 0);
}

/*
 * 找线方向基于未经滤波的原始红外位置，并对最近若干拍做累计投票：
 * 单拍抖动只把计数拨动 1，翻不了方向；丢线前约 200ms 内的主导侧说了算。
 * 不能复用 s_last_error：直线防抖的死区会把小误差置零，导致全白后总落到默认方向。
 */
static void remember_last_line_side(float raw_error)
{
    if (raw_error > LINE_SEARCH_SIDE_MIN_ERROR) {
        if (s_line_side_accum < LINE_SIDE_ACCUM_WINDOW) {
            s_line_side_accum++;
        }
    } else if (raw_error < -LINE_SEARCH_SIDE_MIN_ERROR) {
        if (s_line_side_accum > -LINE_SIDE_ACCUM_WINDOW) {
            s_line_side_accum--;
        }
    }

    if (s_line_side_accum >= LINE_SIDE_FLIP_MIN) {
        s_last_line_side = +1;
    } else if (s_line_side_accum <= -LINE_SIDE_FLIP_MIN) {
        s_last_line_side = -1;
    }
}

static float filter_line_error(float raw_error)
{
    if (!s_error_filter_valid) {
        s_filtered_error = raw_error;
        s_error_filter_valid = true;
    } else {
        s_filtered_error += LINE_ERROR_FILTER_ALPHA * (raw_error - s_filtered_error);
    }

    return (fabsf(s_filtered_error) < LINE_CENTER_DEADBAND) ? 0.0f : s_filtered_error;
}

static bool state_accepts_obstacle_trigger(void)
{
    /*
     * 锐角原地找线期间不接受超声触发：定时长避障动作会打断转弯出口确认，
     * 且旋转中测到的“障碍”常是赛道边物，容易把弯道恢复引入 FAILSAFE_STOP。
     */
    return s_state == CAR_STATE_RUN ||
           s_state == CAR_STATE_LINE_SEARCH;
}

static bool obstacle_confirmed(uint16_t distance_cm, uint32_t now_ms)
{
    if (distance_cm == ULTRASONIC_INVALID) {
        return false;  /* 无新样本时不增加也不清空计数 */
    }

    if (elapsed_ms(now_ms, s_last_distance_sample_ms) > OBSTACLE_SAMPLE_GAP_MS) {
        s_obstacle_samples = 0;
    }
    s_last_distance_sample_ms = now_ms;

    if (distance_cm <= OBSTACLE_TRIGGER_CM) {
        if (s_obstacle_samples < OBSTACLE_CONFIRM_SAMPLES) {
            s_obstacle_samples++;
        }
    } else if (distance_cm >= OBSTACLE_CLEAR_CM) {
        s_obstacle_samples = 0;
    }

    return s_obstacle_samples >= OBSTACLE_CONFIRM_SAMPLES;
}

static void run_line_controller(const infrared_pattern_t *pat, uint32_t now_ms)
{
    float error = 0.0f;
    const bool has_line = infrared_get_error(pat, &error);

    if (!has_line) {
        if (!s_line_lost_timing) {
            s_line_lost_timing = true;
            s_line_lost_since_ms = now_ms;
        }

        /*
         * 锐角进入后出现全白，说明外侧探头已越过拐点。
         * 立刻停止前进，按最后确认的黑线侧原地转；不再用固定毫秒数盲目前冲。
         */
        if (s_last_line_was_corner) {
            s_corner_turn_side = s_last_line_side;
            s_corner_exit_samples = 0;
            s_corner_retry_used = false;
            enter_state(CAR_STATE_CORNER_SEARCH, now_ms);
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
            return;
        }

        if (elapsed_ms(now_ms, s_line_lost_since_ms) < LINE_LOST_GRACE_MS) {
            const int held_turn = clamp_int(
                (int)lroundf(STEERING_SIGN * LINE_KP * s_last_error),
                -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
            apply_body_command(LINE_LOST_FORWARD_PCT, limit_line_turn_slew(held_turn), 0);
        } else {
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
        }
        return;
    }

    s_line_lost_timing = false;

    /* 在滤波/死区前记录方向，供全白后的原地找线使用。 */
    remember_last_line_side(error);

    /* 全黑通常是较宽的线或路口：保持低速直行，不执行未经赛道验证的转向。 */
    if (infrared_all_black(pat)) {
        s_last_error = 0.0f;
        s_filtered_error = 0.0f;
        s_error_filter_valid = true;
        s_last_line_was_corner = false;
        apply_body_command(LINE_MIN_SPEED_PCT, 0, 0);
        return;
    }

    if (infrared_is_sharp_corner(pat)) {
        /*
         * 外侧探头命中是方向的最强证据：直接锁定找线方向并把投票计数
         * 置到对应侧，不等逐拍积累（20ms 采样可能只采到一两拍弯道图样）。
         * 方向锁定对全部弯道图样生效，与控制动作无关。
         */
        s_last_line_side = (error < 0.0f) ? -1 : +1;
        s_line_side_accum = s_last_line_side * LINE_SIDE_FLIP_MIN;
        s_last_line_was_corner = true;

        if (infrared_is_corner_imminent(pat)) {
            /* 外侧单点、马上要丢线：低速+固定大转向，抢在丢线前把车转过来。 */
            s_last_error = error;
            s_filtered_error = error;
            s_error_filter_valid = true;
            const int corner_turn = (error < 0.0f ? -1 : +1) *
                                    STEERING_SIGN * LINE_CORNER_TURN_PCT;
            apply_body_command(LINE_CORNER_SPEED_PCT, limit_line_turn_slew(corner_turn), 0);
            return;
        }
        /* a&&b / c&&d：只是偏得多，不落弯道动作，继续走下面的正常 PD。 */
    } else {
        s_last_line_was_corner = false;
    }

    const float filtered_error = filter_line_error(error);
    const float derivative = filtered_error - s_last_error;
    s_last_error = filtered_error;
    int turn = (int)lroundf(STEERING_SIGN *
                            (LINE_KP * filtered_error + LINE_KD * derivative));
    turn = clamp_int(turn, -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
    turn = limit_line_turn_slew(turn);

    /* 转弯越急，前进速度越低，减少冲出赛道。 */
    int forward = LINE_BASE_SPEED_PCT - abs(turn) / 2;
    forward = clamp_int(forward, LINE_MIN_SPEED_PCT, LINE_BASE_SPEED_PCT);
    apply_body_command(forward, turn, 0);
}

void walking_init(void)
{
    motor_init();
    s_output_enabled = false;
    walking_reset(0);
}

void walking_reset(uint32_t now_ms)
{
    enter_state(CAR_STATE_RUN, now_ms);
    s_line_lost_since_ms = now_ms;
    s_last_distance_sample_ms = now_ms;
    s_obstacle_samples = 0;
    s_line_reacquire_samples = 0;
    s_corner_exit_samples = 0;
    s_line_lost_timing = false;
    s_last_line_was_corner = false;
    s_corner_retry_used = false;
    s_last_error = 0.0f;
    s_last_line_side = +1;
    s_line_side_accum = 0;
    s_corner_turn_side = +1;
    s_filtered_error = 0.0f;
    s_error_filter_valid = false;
    s_last_command = (walking_command_t){0};
    if (s_output_enabled) {
        motor_stop_all();
    }
}

void walking_set_output_enabled(bool enabled)
{
    if (!enabled) {
        motor_stop_all();
        motor_enable(false);
        s_output_enabled = false;
        return;
    }

    motor_enable(true);
    s_output_enabled = true;
}

void walking_update(const infrared_pattern_t *pat,
                    uint16_t distance_cm,
                    uint32_t now_ms)
{
    if (pat == NULL) {
        enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
        apply_brake();
        return;
    }

    if (state_accepts_obstacle_trigger() &&
        obstacle_confirmed(distance_cm, now_ms)) {
        s_obstacle_samples = 0;
        enter_state(CAR_STATE_AVOID_STOP, now_ms);
    }

    switch (s_state) {
    case CAR_STATE_RUN:
        run_line_controller(pat, now_ms);
        break;

    case CAR_STATE_LINE_SEARCH: {
        float error;
        if (infrared_get_error(pat, &error)) {
            if (++s_line_reacquire_samples >= 2) {
                /* 干净重启滤波器，不把丢线前的旧误差带进 RUN。 */
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
                break;
            }
        } else {
            s_line_reacquire_samples = 0;
        }

#if LINE_SEARCH_TIMEOUT_MS > 0u
        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_SEARCH_TIMEOUT_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }
#endif

        /*
         * 普通丢线找线也不在第一次见线时刹车：继续低速原地转，
         * 直到两拍连续确认后才回到巡线，避免黑白临界处反复停转。
         */
        apply_body_command(0, s_last_line_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
        break;
    }

    case CAR_STATE_CORNER_SEARCH: {
        float error;
        const bool has_line = infrared_get_error(pat, &error);

        /*
         * 这里不会因满 1000ms 而结束转弯。它仅说明首选方向尚未找到可信出口，
         * 因而反向尝试一次；此后仍由“连续见线”决定何时恢复巡线。
         * 注意不再重置 s_state_enter_ms：总超时从进入本状态起算。
         */
        if (!s_corner_retry_used &&
            elapsed_ms(now_ms, s_state_enter_ms) >= LINE_CORNER_RETRY_MS) {
            s_corner_turn_side = -s_corner_turn_side;
            s_corner_retry_used = true;
            s_corner_exit_samples = 0;
        }

        if (has_line &&
            infrared_is_corner_exit_candidate(pat, s_corner_turn_side)) {
            if (s_corner_exit_samples < LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                s_corner_exit_samples++;
            }

            if (s_corner_exit_samples >= LINE_CORNER_EXIT_CONFIRM_SAMPLES) {
                /* 干净重启滤波器，不把弯道里的瞬时误差带进 RUN。 */
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                s_last_line_was_corner = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
                break;
            }

            /*
             * 确认期间先停住转动，让探头停在线上完成连续采样；
             * 否则原地旋转稍快时线在探头下停留不足确认窗口，计数被反复清零。
             */
            apply_body_command(0, 0, 0);
        } else {
            s_corner_exit_samples = 0;
            apply_body_command(0,
                               s_corner_turn_side * STEERING_SIGN * LINE_SEARCH_TURN_PCT, 0);
        }

        /* 总兜底：首选+反向都没找到出口，进故障停车而不是无限空转。 */
        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_CORNER_TIMEOUT_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
        }
        break;
    }

    case CAR_STATE_AVOID_STOP:
        apply_brake();
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_STOP_MS) {
            enter_state(CAR_STATE_AVOID_STRAFE_LEFT, now_ms);
        }
        break;

    case CAR_STATE_AVOID_STRAFE_LEFT:
        /* strafe>0 = 左移（方向由 STRAFE_SIGN_* 标定） */
        apply_body_command(0, 0, AVOID_STRAFE_SPEED_PCT);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_STRAFE_LEFT_MS) {
            enter_state(CAR_STATE_AVOID_FORWARD, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FORWARD:
        /* 直行前进，越过障碍 */
        apply_body_command(AVOID_FORWARD_SPEED_PCT, 0, 0);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_FORWARD_MS) {
            enter_state(CAR_STATE_AVOID_STRAFE_RIGHT, now_ms);
        }
        break;

    case CAR_STATE_AVOID_STRAFE_RIGHT:
        /* strafe<0 = 右移（方向由 STRAFE_SIGN_* 标定） */
        apply_body_command(0, 0, -AVOID_STRAFE_SPEED_PCT);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_STRAFE_RIGHT_MS) {
            enter_state(CAR_STATE_AVOID_FIND_LINE, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FIND_LINE:
        if (!infrared_all_white(pat)) {
            s_last_error = 0.0f;
            s_filtered_error = 0.0f;
            s_error_filter_valid = false;
            s_line_lost_timing = false;
            enter_state(CAR_STATE_RUN, now_ms);
            run_line_controller(pat, now_ms);
            break;
        }
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_FIND_LINE_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }
        apply_body_command(AVOID_FIND_SPEED_PCT, -AVOID_SCAN_TURN_SIGN * 5, 0);
        break;

    case CAR_STATE_FAILSAFE_STOP:
    default:
        apply_brake();
        /*
         * 故障停车不是死锁：连续 5 拍稳定见线后自动恢复巡线。
         * 恢复前同样重置滤波器和丢线计时，避免带入停车前的旧状态。
         */
        if (!infrared_all_white(pat)) {
            if (++s_line_reacquire_samples >= 5) {
                s_last_error = 0.0f;
                s_filtered_error = 0.0f;
                s_error_filter_valid = false;
                s_line_lost_timing = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
            }
        } else {
            s_line_reacquire_samples = 0;
        }
        break;
    }
}

car_state_t walking_get_state(void)
{
    return s_state;
}

const char *walking_state_name(car_state_t state)
{
    if ((unsigned)state >= (sizeof(s_state_name) / sizeof(s_state_name[0]))) {
        return "?";
    }
    return s_state_name[state];
}

walking_command_t walking_get_last_command(void)
{
    return s_last_command;
}

float walking_get_last_error(void)
{
    return s_last_error;
}
