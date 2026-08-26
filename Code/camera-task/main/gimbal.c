#include "gimbal.h"
#include <stdio.h>

/* ============================================================
 * 云台实现（MG90S PWM 舵机，软件记录角度）
 *
 * ⚠️ 当前为占位实现：记录角度但不实际输出 PWM。
 *    引脚确定后，用 LEDC 50Hz + 0.5~2.5ms 脉宽替换 servo_write()。
 *
 * 校准逻辑（上电自动）：
 *   1. pan 向 +120° 方向转一段时间（应被物理限位挡住）
 *   2. tilt 向 +90° 方向转一段时间（应被物理限位挡住）
 *   3. 以限位位置为参考，把软件角度设到对应限位值，从而确定机械零位
 *   4. 回到 0° 开始工作
 *
 * 方向标定：gimbal_axis_direction_test() 独立提供，
 *   用于提前实测"脉宽增大 → 物理转向"的关系。
 * ============================================================ */

/* 校准参数 */
#define GIMBAL_CALIB_PAN_STEP_MS   300   /* pan 向限位转的时长（ms） */
#define GIMBAL_CALIB_TILT_STEP_MS  300   /* tilt 向限位转的时长（ms） */
#define GIMBAL_CALIB_HOLD_MS       500   /* 到达限位后的保持时间（ms） */

static int s_pan_pin;
static int s_tilt_pin;
static int s_pan_angle = 0;    /* 当前记录的 pan 角（机械零位=0） */
static int s_tilt_angle = 0;   /* 当前记录的 tilt 角（机械零位=0） */
static bool s_calibrated = false;

/* 占位：真实实现用 LEDC 输出 PWM 脉宽（0.5ms~2.5ms 对应 0~180°） */
static void servo_write(int pin, int angle_deg)
{
    if (pin == 0) return;   /* 未接线 */
    /* TODO: ledc 输出 50Hz，duty = (0.5 + angle/180*2.0)ms 对应脉宽 */
    printf("[gimbal] pin=%d 舵机→%d°\n", pin, angle_deg);
}

/* 简单延时（占位用 vTaskDelay 更好，但这里保持轻量） */
static void delay_ms(int ms)
{
    for (volatile int i = 0; i < ms * 10000; i++) {
    }
}

/* 校准单个轴：向 +限位 方向转指定时长 */
static void calib_push_to_limit(int pin, int *angle_out, int limit, int dur_ms)
{
    *angle_out = limit;              /* 先假设已到限位 */
    servo_write(pin, limit);         /* 发指令：向限位转 */
    delay_ms(dur_ms);                /* 让它转到被物理挡住 */
    *angle_out = limit;              /* 确认：软件认为停在限位 */
}

/* ============================================================
 * 舵机方向测试（独立测试入口，用于提前标定安装方向）
 *
 * 原理：直接以"原始舵机角度"（0~180°）驱动舵机（绕开任何
 * 软件坐标换算），依次输出 0° → 180° → 0°，让用户观察每个
 * 舵机的物理转向：
 *   - pan 舵机：脉宽从 0.5ms 增到 2.5ms（0°→180°）时云台往哪转？
 *   - tilt 舵机：同上，云台往哪转？
 *
 * 用法：接线后单独运行本函数，记录结果，据此在后续
 *   控制代码中做"物理角度 ↔ 舵机脉宽"的符号映射。
 *
 * 注意：本函数不依赖 s_pan_angle/s_tilt_angle，可独立反复运行。
 * ============================================================ */
void gimbal_axis_direction_test(void)
{
    printf("\n[gimbal] === 舵机方向测试（观察物理转向） ===\n");
    printf("[gimbal] 目的：实测'脉宽增大 → 舵机转向'的对应关系。\n\n");

    /* pan 测试 */
    printf("[gimbal] ---- pan（左右）----\n");
    printf("[gimbal] 即将从 0°(0.5ms) → 180°(2.5ms) 摆动\n");
    printf("[gimbal] 请观察：脉宽增大时，云台向 左 还是 右 转？\n");
    for (int a = 0; a <= 180; a += 30) {
        servo_write(s_pan_pin, a);
        delay_ms(300);
    }
    delay_ms(500);
    for (int a = 180; a >= 0; a -= 30) {
        servo_write(s_pan_pin, a);
        delay_ms(300);
    }
    printf("[gimbal] pan 测试结束。请记录：\n");
    printf("[gimbal]   脉宽增大(0.5→2.5ms) 时 pan 向___转（左/右）\n\n");

    /* tilt 测试 */
    printf("[gimbal] ---- tilt（上下）----\n");
    printf("[gimbal] 即将从 0°(0.5ms) → 180°(2.5ms) 摆动\n");
    printf("[gimbal] 请观察：脉宽增大时，云台向 上 还是 下 转？\n");
    for (int a = 0; a <= 180; a += 30) {
        servo_write(s_tilt_pin, a);
        delay_ms(300);
    }
    delay_ms(500);
    for (int a = 180; a >= 0; a -= 30) {
        servo_write(s_tilt_pin, a);
        delay_ms(300);
    }
    printf("[gimbal] tilt 测试结束。请记录：\n");
    printf("[gimbal]   脉宽增大(0.5→2.5ms) 时 tilt 向___转（上/下）\n\n");
    printf("[gimbal] === 方向测试完成 ===\n");
    printf("[gimbal] 根据记录结果，在控制代码中确定物理角度↔脉宽映射。\n");
}

void gimbal_init(int pan_pin, int tilt_pin)
{
    s_pan_pin = pan_pin;
    s_tilt_pin = tilt_pin;
    s_calibrated = false;
    s_pan_angle = 0;
    s_tilt_angle = 0;

    printf("[gimbal] 开始上电校准（pan/tilt 先向限位转，确定机械零位）...\n");

    /* 1. pan 向 +120° 限位转（应被物理挡停） */
    calib_push_to_limit(s_pan_pin, &s_pan_angle, GIMBAL_PAN_LIMIT_MAX,
                        GIMBAL_CALIB_PAN_STEP_MS);
    /* 2. tilt 向 +90° 限位转 */
    calib_push_to_limit(s_tilt_pin, &s_tilt_angle, GIMBAL_TILT_LIMIT_MAX,
                        GIMBAL_CALIB_TILT_STEP_MS);
    /* 3. 短暂保持（确认堵在限位） */
    delay_ms(GIMBAL_CALIB_HOLD_MS);

    /* 4. 回到机械零位（0°）开始工作 */
    gimbal_set_pan(0);
    gimbal_set_tilt(0);

    s_calibrated = true;
    printf("[gimbal] 校准完成：pan=0°, tilt=0°（机械零位）\n");
}

int gimbal_set_pan(int angle_deg)
{
    /* 限位夹紧 */
    if (angle_deg < GIMBAL_PAN_LIMIT_MIN)  angle_deg = GIMBAL_PAN_LIMIT_MIN;
    if (angle_deg > GIMBAL_PAN_LIMIT_MAX)  angle_deg = GIMBAL_PAN_LIMIT_MAX;
    s_pan_angle = angle_deg;
    servo_write(s_pan_pin, s_pan_angle);
    return s_pan_angle;
}

int gimbal_set_tilt(int angle_deg)
{
    /* 限位夹紧 */
    if (angle_deg < GIMBAL_TILT_LIMIT_MIN)  angle_deg = GIMBAL_TILT_LIMIT_MIN;
    if (angle_deg > GIMBAL_TILT_LIMIT_MAX)  angle_deg = GIMBAL_TILT_LIMIT_MAX;
    s_tilt_angle = angle_deg;
    servo_write(s_tilt_pin, s_tilt_angle);
    return s_tilt_angle;
}

int gimbal_get_pan(void)
{
    return s_pan_angle;
}

int gimbal_get_tilt(void)
{
    return s_tilt_angle;
}

bool gimbal_is_calibrated(void)
{
    return s_calibrated;
}
