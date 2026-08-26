#include "motion.h"
#include <stdio.h>

/* ============================================================
 * 电机控制实现（暂定占位）
 *
 * ⚠️ 当前版本不驱动真实电机，仅打印指令便于联调观察。
 * 接入真实运动学后替换本文件即可，motion.h 接口保持不变。
 * ============================================================ */

static bool s_inited = false;

void motion_init(void)
{
    s_inited = true;
    printf("[motion] 电机驱动占位实现（未实际驱动）。\n");
}

void motion_set_left_right(int left, int right)
{
    if (!s_inited) return;
    printf("[motion] 差速 L=%d R=%d\n", left, right);
}

void motion_rotate(int speed)
{
    if (!s_inited) return;
    printf("[motion] 原地旋转 speed=%d\n", speed);
}

void motion_move_toward(int angle_deg, int speed)
{
    if (!s_inited) return;
    printf("[motion] 朝角 %d° 前进 speed=%d\n", angle_deg, speed);
}

void motion_stop(void)
{
    if (!s_inited) return;
    printf("[motion] 停车\n");
}
