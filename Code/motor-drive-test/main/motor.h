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
    MOTOR_DIR_BRAKE = 2,    // IN1=0, IN2=0  (short brake)
    MOTOR_DIR_COAST = 3,    // IN1=1, IN2=1  (stop/coast)
} motor_dir_t;

/**
 * @brief 初始化三路电机：STBY 使能、方向 GPIO、LEDC PWM (20kHz, 10bit)
 */
void motor_init(void);

/**
 * @brief 设置电机速度（带方向）
 * @param id       MOTOR_A / MOTOR_B / MOTOR_C
 * @param percent  -100 ~ +100（正=正转，负=反转，0=停止）
 */
void motor_set_speed(int id, int percent);

/**
 * @brief 直接设置方向（PWM 保持原值）
 */
void motor_set_dir(int id, motor_dir_t dir);

/**
 * @brief 停止所有电机（刹车）
 */
void motor_stop_all(void);
