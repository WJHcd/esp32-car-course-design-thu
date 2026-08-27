#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "motor.h"

/* 三路电机的真实转速快照；rpm 为编码器测得的转速大小，不是 PWM 百分比。 */
typedef struct {
    float rpm[MOTOR_NUM];
    bool valid[MOTOR_NUM];
} encoder_rpm_t;

/** 初始化三路 PCNT 脉冲计数器。初始化后不驱动电机。 */
void encoder_init(void);

/**
 * 读取并更新三路 RPM。
 * 建议每 200~300ms 调用一次；第一次调用仅建立基准，valid 为 false。
 */
void encoder_update(void);

/** 获取最近一次 RPM 快照。 */
encoder_rpm_t encoder_get_rpm(void);
