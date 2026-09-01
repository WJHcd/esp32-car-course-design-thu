#pragma once
#include <stdint.h>
#include <stdbool.h>

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_NUM 3

typedef enum {
    MOTOR_DIR_FORWARD = 0,  // IN1=1, IN2=0
    MOTOR_DIR_BACKWARD = 1, // IN1=0, IN2=1
    MOTOR_DIR_BRAKE = 2,    // IN1=1, IN2=1  (short brake)
    MOTOR_DIR_COAST = 3,    // IN1=0, IN2=0  (stop/high impedance)
} motor_dir_t;

/**
 * @brief 初始化三路电机；初始化完成后 STBY 保持低，电机不会动作
 */
void motor_init(void);

/**
 * @brief 控制 D24A 全局使能；禁用前会先将三路 PWM 清零并设为滑行
 */
void motor_enable(bool enable);

/**
 * @brief 查询当前是否已使能 D24A
 */
bool motor_is_enabled(void);

/**
 * @brief 设置电机速度（带方向）
 * @param id       MOTOR_A / MOTOR_B / MOTOR_C
 * @param percent  -100 ~ +100，支持小数（PWM 10bit，约 0.1% 分辨率，
 *                 供 0.5% 步进的现场微调；正=正转，负=反转，0=滑行停止）
 */
void motor_set_speed(int id, float percent);

/**
 * @brief 直接设置方向（PWM 保持原值）
 */
void motor_set_dir(int id, motor_dir_t dir);

/**
 * @brief PWM 清零并将所有电机设为短刹车；不改变 STBY 状态
 */
void motor_stop_all(void);
