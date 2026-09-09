#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 电机接口（真实驱动：D24A 三路 + 三轮全向混控）
 *
 * 底盘为三轮全向（电机 A/B/C，D24A 驱动板），上层不再关心
 * 单轮速度，只下发"分量指令"（与 task1/task2 walking.c 一致）：
 *
 *   - fwd    前向分量（正=前进）
 *   - turn   旋转分量（正=逆时针自转）
 *   - strafe 横移分量（正=向左平移）
 *
 * 范围 -100~100，为**相对权重**：单轴满量程/多轴峰值统一归一化
 * 到轮速上限 WHEEL_MAX_PCT(30%)，混控方向/轮增益为 task2
 * 实车标定值（见 motion.c），零输入即停。
 *
 * ⚠️ motion_init() 后 STBY 保持低电平、电机不会动作；
 *    必须先 motion_enable(true) 才允许 motion_drive()。
 *    驱动层做了 <1% 指令按滑行停止、换向先清 PWM 滑行 200us
 *    等保护（移植自 task3-ble-control 的 motor.c，2026-09 实车验证）。
 * ============================================================ */

/**
 * @brief 初始化 D24A 电机驱动：STBY 拉低 + 方向脚 + LEDC(20kHz/10bit)
 */
void motion_init(void);

/**
 * @brief D24A 全局使能（STBY）；禁用前先清 PWM 并滑行
 * @param enable true=使能
 */
void motion_enable(bool enable);

/** @brief 当前是否已使能 D24A */
bool motion_is_enabled(void);

/**
 * @brief 全向驱动：按分量混控到 A/B/C 三路轮速
 * @param fwd    前向分量 -100~100（正=前进）
 * @param turn   旋转分量 -100~100（正=逆时针）
 * @param strafe 横移分量 -100~100（正=左移）
 *
 * 使能(STBY)由上层负责（各遥控模式进入时 motion_enable(true)）。
 * 未使能时 motion_drive() 直接忽略指令——STBY 低时 D24A 输出
 * 高阻不会转动，使能位即总开关。
 */
void motion_drive(int fwd, int turn, int strafe);

/**
 * @brief 全停：PWM 清零 + 三路短刹车（不改变 STBY 使能状态）
 */
void motion_stop(void);
