#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 电机控制接口（与 camera-task 保持一致）
 *
 * 说明：三轮全向底盘的运动学最终由运动学模块实现，
 * 这里先定义上层（语音动作执行器）调用所需的抽象接口。
 * 实现占位：motion.c 仅打印，不驱动电机。
 * 后续接入真实运动学时，只需替换实现，接口保持不变。
 *
 * 约定：
 *   - 速度范围 [-100, 100]（正=前进/正向）
 *   - 角度单位：度；0°=正前方，逆时针为正（x 轴向右）
 * ============================================================ */

/**
 * @brief 初始化电机驱动（占位：仅打印）
 */
void motion_init(void);

/**
 * @brief 差速前进/修正：左轮、右轮速度（-100~100）
 * @param left   左轮速度
 * @param right  右轮速度
 */
void motion_set_left_right(int left, int right);

/**
 * @brief 原地旋转
 * @param speed   旋转速度（-100~100，正=逆时针）
 */
void motion_rotate(int speed);

/**
 * @brief 向指定方位角移动（供"接近球"使用）
 * @param angle_deg  目标方位角（度，0=正前方）
 * @param speed      前进速度（-100~100）
 */
void motion_move_toward(int angle_deg, int speed);

/**
 * @brief 全停
 */
void motion_stop(void);
