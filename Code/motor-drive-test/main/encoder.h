#pragma once
#include <stdint.h>

/**
 * @brief 初始化 3 路霍尔编码器（PCNT 外设，A 双边沿 + B 方向 → 2 倍频）
 *        512 线 → 每圈 1024 计数
 */
void encoder_init(void);

/**
 * @brief 读取指定电机编码器累计计数
 * @param id MOTOR_A / MOTOR_B / MOTOR_C
 */
int32_t encoder_get_count(int id);

/** @brief 清零指定编码器累计计数。 */
void encoder_clear_count(int id);

/** @brief 清零三个编码器累计计数。 */
void encoder_clear_all(void);
