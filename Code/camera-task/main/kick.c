#include "kick.h"
#include <stdio.h>

/* ============================================================
 * 踢球机构实现（占位）
 *
 * 真实实现：LEDC 50Hz PWM，kick 时舵机从收回角快速甩到踢出角。
 * ============================================================ */

static int s_kick_pin;

void kick_init(int pin)
{
    s_kick_pin = pin;
    printf("[kick] 踢球舵机占位实现（pin=%d，未实际驱动）。\n", pin);
}

void kick_execute(void)
{
    printf("[kick] 执行踢球动作！\n");
    /* TODO: 舵机快速甩动（LEDC 脉宽切换） */
}

void kick_reset(void)
{
    printf("[kick] 踢球机构复位\n");
    /* TODO: 舵机回到收回位置 */
}
