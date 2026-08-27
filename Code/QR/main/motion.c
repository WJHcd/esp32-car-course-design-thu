#include "motion.h"
#include "motor.h"
#include <stdio.h>

/* ============================================================
 * 原地旋转（复用 task1 标定的三轮符号）
 *
 * task1 结论（KS-04 实车）：
 *   A=左轮、C=右轮、B=后轮；
 *   同一 PWM 下 C 轮略快于 A → C 乘 0.90 系数；
 *   TURN 分量：A/C 同向、B 反向 → 车身原地旋转。
 *
 * ⚠️ 方向标定：motion_rotate(+speed) 是顺时针还是逆时针，
 *    上实车后如与预期相反，把 MOTION_ROTATE_SIGN 改成 -1。
 * ============================================================ */

#define MOTION_ROTATE_SIGN  (+1)
#define MOTION_C_GAIN       0.90f   /* C(右) 轮速度补偿，同 task1 */

static bool s_inited = false;

void motion_init(void)
{
    motor_init();
    motor_enable(true);
    s_inited = true;
    printf("[motion] 电机已使能（原地旋转模式）。\n");
}

void motion_rotate(int speed)
{
    if (!s_inited) return;
    if (speed > 100)  speed = 100;
    if (speed < -100) speed = -100;

    if (speed == 0) {
        motor_stop_all();
        return;
    }

    const int s = MOTION_ROTATE_SIGN * speed;
    motor_set_speed(MOTOR_A, s);                        /* 左轮 */
    motor_set_speed(MOTOR_C, (int)(s * MOTION_C_GAIN)); /* 右轮 */
    motor_set_speed(MOTOR_B, -s);                       /* 后轮反向 */
}

void motion_stop(void)
{
    if (!s_inited) return;
    motor_stop_all();
}
