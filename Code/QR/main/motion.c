#include "motion.h"
#include "motor.h"
#include <stdio.h>

/* ============================================================
 * 原地旋转（对齐 WJHcd 仓库 task1_ver2.0 walking.c 实车标定）
 *
 * 实车结构（task1_ver2.0 已确认）：B 在正后方，A 在右前方，
 * C 在左前方，三驱动轮朝外互成 120°。
 *
 * 转向分量（实车已验证）：A=+turn、C=+turn、B=-0.68*turn——
 *   ⚠️ B 电机"正转"方向约定与 A/C 相反（安装镜像所致），且同一
 *   指令下 B 轮偏快，0.68 为其实测折中（walking.c TURN_B_GAIN）。
 *
 * ⚠️ 方向标定：motion_rotate(+speed) 是顺时针还是逆时针，
 *    上实车后如与预期相反，把 MOTION_ROTATE_SIGN 改成 -1。
 * ============================================================ */

#define MOTION_ROTATE_SIGN  (+1)
#define MOTION_TURN_GAIN_B  0.68f   /* B 轮速度补偿，同 task1_ver2.0 TURN_B_GAIN */

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
    motor_set_speed(MOTOR_A, s);                        /* 右前轮 */
    motor_set_speed(MOTOR_C, s);                        /* 左前轮（与 A 同速） */
    motor_set_speed(MOTOR_B, (int)(-s * MOTION_TURN_GAIN_B)); /* 后轮反向 + 增益补偿 */
}

void motion_stop(void)
{
    if (!s_inited) return;
    motor_stop_all();
}
