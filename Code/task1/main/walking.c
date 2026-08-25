#include "walking.h"
#include "motor.h"
#include "ultrasonic.h"

/* ============================================================
 * 行走策略实现
 * 逻辑逐行移植自参考 STM32 程序 car_tim()，并做了三处适配：
 *   1. 参考程序用定时器中断每次读传感器；这里改为由 main.c 传入
 *      infrared_read() 的结果与 ultrasonic 距离。
 *   2. 参考程序 PWM 量程 ±6000，本驱动 motor_set_speed() 是 ±100，
 *      用 pwm_to_percent() 归一化。
 *   3. 参考程序三轮 L(左)/B(后)/R(右)，这里映射到本车电机 A/B/C，
 *      装车方向不同时改 WHEEL_TO_MOTOR_* 即可。
 * ============================================================ */

/* ---------- 轮子 → 电机 映射（按实际装车方向调整） ---------- */
#define WHEEL_TO_MOTOR_L   MOTOR_A
#define WHEEL_TO_MOTOR_R   MOTOR_B
#define WHEEL_TO_MOTOR_B   MOTOR_C

/* 参考程序 PWM 量程 ±6000 → motor_set_speed ±100 */
#define REF_PWM_MAX        6000

/* ---------- 状态名表（顺序必须与 car_state_t 一致） ---------- */
static const char *s_state_name[] = {
    "RUN", "MEET_FACE", "BACKWARD", "SEARCH_EXIT_LEFT",
    "SEARCH_EXIT_RIGHT", "BACK_TO_FORWARD", "DASH", "AVOIDING", "ROTATE",
};

/* ---------- PID 参数（沿用参考值，Kp=500 对应参考 PWM 量程） ---------- */
static float s_kp = 500.0f;
static float s_ki = 0.0f;
static float s_kd = 0.0f;
static float s_integral = 0.0f;
static float s_prev_error = 0.0f;

/* ---------- 状态机变量 ---------- */
static car_state_t s_state = CAR_STATE_RUN;
static car_state_t s_next_state = CAR_STATE_RUN;
static int s_car_v = 0;    /* 前进/后退基础速度（参考量程 ±6000） */
static int s_output = 0;   /* PID/转向输出 */

/* ---------- 状态计数器（参考程序的常量/变量） ---------- */
#define BACKWARD_TIMES_MAX          2
#define LEFT_ROTATE_TIMES_MAX       6
#define RIGHT_ROTATE_TIMES_MAX      12
#define ROTATE_BACKWARD_TIMES_MAX   2
#define BACK_TO_FORWARD_TIMES_MAX   6
#define DASH_TIMES_MAX              3
#define ROTATE_TIME_MAX             23
#define BACK_TO_ORIGINAL_TIME_MAX   13

static int s_backward_times;
static int s_left_rotate_times;
static int s_right_rotate_times;
static int s_rotate_backward_times;
static int s_back_to_forward_times;
static int s_dash_times;
static int s_rotate_time;
static int s_back_to_original_time;

static bool s_output_enabled = true;

/* 参考 PWM 值 → motor_set_speed 的百分数 */
static int pwm_to_percent(int pwm)
{
    if (pwm >  REF_PWM_MAX) pwm =  REF_PWM_MAX;
    if (pwm < -REF_PWM_MAX) pwm = -REF_PWM_MAX;
    return pwm * 100 / REF_PWM_MAX;
}

/* 按参考程序的三轮 L/R/B 驱动电机 */
static void drive_wheels(int L, int R, int B)
{
    if (!s_output_enabled) {
        return;
    }
    motor_set_speed(WHEEL_TO_MOTOR_L, pwm_to_percent(L));
    motor_set_speed(WHEEL_TO_MOTOR_R, pwm_to_percent(R));
    motor_set_speed(WHEEL_TO_MOTOR_B, pwm_to_percent(B));
}

void walking_init(void)
{
    motor_init();

    s_state = CAR_STATE_RUN;
    s_next_state = CAR_STATE_RUN;
    s_car_v = 0;
    s_output = 0;
    s_integral = 0.0f;
    s_prev_error = 0.0f;

    s_backward_times = BACKWARD_TIMES_MAX;
    s_left_rotate_times = LEFT_ROTATE_TIMES_MAX;
    s_right_rotate_times = RIGHT_ROTATE_TIMES_MAX;
    s_rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;
    s_back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;
    s_dash_times = DASH_TIMES_MAX;
    s_rotate_time = ROTATE_TIME_MAX;
    s_back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
}

void walking_update(const infrared_pattern_t *pat, uint16_t distance_cm)
{
    /* ---- 超声波避障触发：距离 ≤ 9cm 且非无效、非避障/旋转中 ---- */
    if (distance_cm <= 9 && distance_cm != ULTRASONIC_INVALID &&
        s_state != CAR_STATE_AVOIDING && s_state != CAR_STATE_ROTATE) {
        s_state = CAR_STATE_AVOIDING;
        s_rotate_time = ROTATE_TIME_MAX;
        s_back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
    }

    const int a = pat->a, b = pat->b, c = pat->c, d = pat->d;

    switch (s_state) {
    case CAR_STATE_RUN: {
        /* 参考程序：E_V = (a*1.9 + b*1.2) - (c*1.2 + d*1.9) */
        float ev = (a * 1.9f + b * 1.2f) - (c * 1.2f + d * 1.9f);

        /* 只有最外侧单边在线时误差清零（避免无谓转向） */
        if (a == 0 && b == 0 && c == 0 && d == -1) ev = 0.0f;
        else if (a == -1 && b == 0 && c == 0 && d == 0) ev = 0.0f;

        /* PID（Ki=0，只有比例项） */
        s_integral += ev;
        if (s_integral > 1000)  s_integral = 1000;
        if (s_integral < -1000) s_integral = -1000;
        float derivative = ev - s_prev_error;
        s_output = (int)(s_kp * ev + s_ki * s_integral + s_kd * derivative);
        s_prev_error = ev;

        /* 直行速度 800；转向时前进速度为 0 */
        s_car_v = (ev == 0.0f) ? 800 : 0;

        /* 四路全黑 → 遇线/路口 */
        if (a == 0 && b == 0 && c == 0 && d == 0) {
            s_car_v = 0;
            s_next_state = CAR_STATE_MEET_FACE;
        }
        break;
    }

    case CAR_STATE_MEET_FACE:
        s_next_state = CAR_STATE_SEARCH_EXIT_LEFT;
        s_left_rotate_times = LEFT_ROTATE_TIMES_MAX;
        s_right_rotate_times = RIGHT_ROTATE_TIMES_MAX;
        s_back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;
        s_dash_times = DASH_TIMES_MAX;
        break;

    case CAR_STATE_BACKWARD:
        if (s_backward_times > 0) {
            s_backward_times--;
            s_car_v = -800;      /* 后退 */
            s_output = 0;
            s_next_state = CAR_STATE_SEARCH_EXIT_LEFT;
        } else {
            s_backward_times = BACKWARD_TIMES_MAX;
            s_next_state = CAR_STATE_SEARCH_EXIT_LEFT;
        }
        break;

    case CAR_STATE_SEARCH_EXIT_LEFT:
        if (s_left_rotate_times > 0) {
            s_left_rotate_times--;
            s_car_v = 0;
            s_output = -800;     /* 左转 */
            s_next_state = CAR_STATE_SEARCH_EXIT_LEFT;
            s_rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;
        } else if (a == 0 && b == 0 && c == 0 && d == 0) {
            s_next_state = CAR_STATE_SEARCH_EXIT_RIGHT;   /* 仍然全黑 → 向右找 */
        } else if (s_rotate_backward_times > 0) {
            s_rotate_backward_times--;
            s_car_v = -800;
            s_output = 0;
            s_next_state = CAR_STATE_SEARCH_EXIT_LEFT;
        } else {
            s_next_state = CAR_STATE_RUN;   /* 找到出口 */
        }
        break;

    case CAR_STATE_SEARCH_EXIT_RIGHT:
        if (s_right_rotate_times > 0) {
            s_right_rotate_times--;
            s_car_v = 0;
            s_output = 800;      /* 右转 */
            s_next_state = CAR_STATE_SEARCH_EXIT_RIGHT;
            s_rotate_backward_times = ROTATE_BACKWARD_TIMES_MAX;
        } else if (a == 0 && b == 0 && c == 0 && d == 0) {
            s_next_state = CAR_STATE_BACK_TO_FORWARD;    /* 仍然全黑 → 回前进 */
        } else if (s_rotate_backward_times > 0) {
            s_rotate_backward_times--;
            s_car_v = -800;
            s_output = 0;
            s_next_state = CAR_STATE_SEARCH_EXIT_RIGHT;
        } else {
            s_next_state = CAR_STATE_RUN;
        }
        break;

    case CAR_STATE_BACK_TO_FORWARD:
        if (s_back_to_forward_times > 0) {
            s_back_to_forward_times--;
            s_car_v = 0;
            s_output = -800;     /* 右转调整回正 */
            s_next_state = CAR_STATE_BACK_TO_FORWARD;
        } else {
            s_next_state = CAR_STATE_DASH;
            s_back_to_forward_times = BACK_TO_FORWARD_TIMES_MAX;
        }
        break;

    case CAR_STATE_DASH:
        if (s_dash_times > 0) {
            s_dash_times--;
            s_car_v = 1000;      /* 冲刺 */
            s_output = 0;
            s_next_state = CAR_STATE_DASH;
        } else {
            s_next_state = CAR_STATE_RUN;
        }
        break;

    case CAR_STATE_AVOIDING:
        if (s_rotate_time > 0) {
            s_rotate_time--;
            /* 参考：L=0, R=-800, B=1600（原地打转躲开） */
            drive_wheels(0, -800, 1600);
            s_next_state = CAR_STATE_AVOIDING;
        } else {
            s_rotate_time = ROTATE_TIME_MAX;
            s_back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
            s_next_state = CAR_STATE_ROTATE;
        }
        break;

    case CAR_STATE_ROTATE:
        if (s_back_to_original_time > 0) {
            s_back_to_original_time--;
            drive_wheels(-800, -800, -800);   /* 三轮同退回位 */
            s_next_state = CAR_STATE_ROTATE;
        } else {
            drive_wheels(0, 0, 0);
            s_next_state = CAR_STATE_RUN;
            s_back_to_original_time = BACK_TO_ORIGINAL_TIME_MAX;
        }
        break;

    default:
        break;
    }

    /* 更新状态 */
    s_state = s_next_state;

    /* 非避障/旋转状态：由 car_v + output 合成三路 PWM（参考程序逻辑） */
    if (s_state != CAR_STATE_AVOIDING && s_state != CAR_STATE_ROTATE) {
        int L = s_car_v + s_output;
        int R = -s_car_v + s_output;
        int B = (int)(s_output * 1.15f);
        drive_wheels(L, R, B);
    }
}

car_state_t walking_get_state(void)
{
    return s_state;
}

void walking_set_output_enabled(bool en)
{
    s_output_enabled = en;
}

const char *walking_state_name(car_state_t s)
{
    if ((int)s < 0 || (int)s > CAR_STATE_ROTATE) {
        return "?";
    }
    return s_state_name[s];
}
