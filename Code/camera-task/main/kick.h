#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 踢球机构接口（MG90S 舵机）
 *
 * 由 kick_init(pin) 指定舵机引脚；kick_execute() 触发踢球动作。
 * ⚠️ 当前为占位实现（仅打印），引脚/动作时序待定。
 * ============================================================ */

/**
 * @brief 初始化踢球舵机
 * @param pin 舵机 PWM GPIO（0=暂不接线）
 */
void kick_init(int pin);

/**
 * @brief 执行一次踢球动作（快速甩动舵机）
 */
void kick_execute(void);

/**
 * @brief 踢球机构复位（收回，准备下一次）
 */
void kick_reset(void);
