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

/* 若误差为正（黑线在车体右侧）时实车反而向左修正，把它改成 -1。 */
#define STEERING_SIGN       (+1)

/* 无左右避障传感器时只能固定选择一侧。+1/-1 对应哪侧需架空轮子实测。 */
#define AVOID_TURN_SIGN     (+1)

/* ------------------------ 控制参数区 ------------------------ */
#define LINE_BASE_SPEED_PCT       12
#define LINE_MIN_SPEED_PCT        10
#define LINE_MAX_TURN_PCT         13
#define LINE_KP                    4.5f
#define LINE_KD                    1.2f
#define LINE_LOST_GRACE_MS       120u
#define LINE_SEARCH_TIMEOUT_MS  2200u
#define LINE_SEARCH_TURN_PCT      10

#define OBSTACLE_TRIGGER_CM       18u
#define OBSTACLE_CLEAR_CM         24u
#define OBSTACLE_CONFIRM_SAMPLES   2u
#define OBSTACLE_SAMPLE_GAP_MS    350u

#define AVOID_STOP_MS            120u
#define AVOID_BACKUP_MS          300u
#define AVOID_TURN_OUT_MS        500u
#define AVOID_BYPASS_MS          700u
#define AVOID_TURN_BACK_MS       500u
#define AVOID_FIND_LINE_MS      1800u
#define AVOID_BACKUP_SPEED_PCT    10
#define AVOID_TURN_SPEED_PCT      12
#define AVOID_BYPASS_SPEED_PCT    11
#define AVOID_FIND_SPEED_PCT      10

static const char *const s_state_name[] = {
    "RUN",
    "LINE_SEARCH",
    "AVOID_STOP",
    "AVOID_BACKUP",
    "AVOID_TURN_OUT",
    "AVOID_BYPASS",
    "AVOID_TURN_BACK",
    "AVOID_FIND_LINE",
    "FAILSAFE_STOP",
};

static car_state_t s_state;
static uint32_t s_state_enter_ms;
static uint32_t s_line_lost_since_ms;
static uint32_t s_last_distance_sample_ms;
static uint8_t s_obstacle_samples;
static uint8_t s_line_reacquire_samples;
static bool s_line_lost_timing;
static bool s_output_enabled;
static float s_last_error;
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
 * 当前底盘在“不横移”场景下的 L/R/B 混控：
 *   纯前进（已实车校正）：L=-v, R=+v, B=0
 *   原地旋转：左右轮按同向转向分量、后轮 B 按反向分量（1.15 倍补偿）
 * 三轮比例超出 100% 时等比缩放，避免逐轮截断改变运动方向。
 */
static void apply_body_command(int forward, int turn)
{
    s_last_command.forward_percent = forward;
    s_last_command.turn_percent = turn;

    float wheel[MOTOR_NUM] = {0};
    wheel[WHEEL_TO_MOTOR_L] = (float)(FORWARD_SIGN_L * forward + TURN_SIGN_L * turn);
    wheel[WHEEL_TO_MOTOR_R] = (float)(FORWARD_SIGN_R * forward + TURN_SIGN_R * turn);
    wheel[WHEEL_TO_MOTOR_B] = (float)(TURN_SIGN_B * turn) * 1.15f;

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
    if (s_output_enabled) {
        motor_stop_all();
    }
}

static bool state_accepts_obstacle_trigger(void)
{
    return s_state == CAR_STATE_RUN || s_state == CAR_STATE_LINE_SEARCH;
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

        if (elapsed_ms(now_ms, s_line_lost_since_ms) < LINE_LOST_GRACE_MS) {
            const int held_turn = clamp_int(
                (int)lroundf(STEERING_SIGN * LINE_KP * s_last_error),
                -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);
            apply_body_command(LINE_MIN_SPEED_PCT, held_turn);
        } else {
            enter_state(CAR_STATE_LINE_SEARCH, now_ms);
        }
        return;
    }

    s_line_lost_timing = false;

    /* 全黑通常是较宽的线或路口：保持低速直行，不执行未经赛道验证的转向。 */
    if (infrared_all_black(pat)) {
        s_last_error = 0.0f;
        apply_body_command(LINE_MIN_SPEED_PCT, 0);
        return;
    }

    const float derivative = error - s_last_error;
    s_last_error = error;
    int turn = (int)lroundf(STEERING_SIGN * (LINE_KP * error + LINE_KD * derivative));
    turn = clamp_int(turn, -LINE_MAX_TURN_PCT, LINE_MAX_TURN_PCT);

    /* 转弯越急，前进速度越低，减少冲出赛道。 */
    int forward = LINE_BASE_SPEED_PCT - abs(turn) / 3;
    forward = clamp_int(forward, LINE_MIN_SPEED_PCT, LINE_BASE_SPEED_PCT);
    apply_body_command(forward, turn);
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
    s_line_lost_timing = false;
    s_last_error = 0.0f;
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
                s_last_error = error;
                s_line_lost_timing = false;
                enter_state(CAR_STATE_RUN, now_ms);
                run_line_controller(pat, now_ms);
                break;
            }
        } else {
            s_line_reacquire_samples = 0;
        }

        if (elapsed_ms(now_ms, s_state_enter_ms) >= LINE_SEARCH_TIMEOUT_MS) {
            enter_state(CAR_STATE_FAILSAFE_STOP, now_ms);
            apply_brake();
            break;
        }

        const int search_sign = (s_last_error < 0.0f) ? -1 : +1;
        apply_body_command(0, search_sign * STEERING_SIGN * LINE_SEARCH_TURN_PCT);
        break;
    }

    case CAR_STATE_AVOID_STOP:
        apply_brake();
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_STOP_MS) {
            enter_state(CAR_STATE_AVOID_BACKUP, now_ms);
        }
        break;

    case CAR_STATE_AVOID_BACKUP:
        apply_body_command(-AVOID_BACKUP_SPEED_PCT, 0);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_BACKUP_MS) {
            enter_state(CAR_STATE_AVOID_TURN_OUT, now_ms);
        }
        break;

    case CAR_STATE_AVOID_TURN_OUT:
        apply_body_command(0, AVOID_TURN_SIGN * AVOID_TURN_SPEED_PCT);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_TURN_OUT_MS) {
            enter_state(CAR_STATE_AVOID_BYPASS, now_ms);
        }
        break;

    case CAR_STATE_AVOID_BYPASS:
        apply_body_command(AVOID_BYPASS_SPEED_PCT, 0);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_BYPASS_MS) {
            enter_state(CAR_STATE_AVOID_TURN_BACK, now_ms);
        }
        break;

    case CAR_STATE_AVOID_TURN_BACK:
        apply_body_command(0, -AVOID_TURN_SIGN * AVOID_TURN_SPEED_PCT);
        if (elapsed_ms(now_ms, s_state_enter_ms) >= AVOID_TURN_BACK_MS) {
            enter_state(CAR_STATE_AVOID_FIND_LINE, now_ms);
        }
        break;

    case CAR_STATE_AVOID_FIND_LINE:
        if (!infrared_all_white(pat)) {
            s_last_error = 0.0f;
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
        apply_body_command(AVOID_FIND_SPEED_PCT, -AVOID_TURN_SIGN * 5);
        break;

    case CAR_STATE_FAILSAFE_STOP:
    default:
        apply_brake();
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
