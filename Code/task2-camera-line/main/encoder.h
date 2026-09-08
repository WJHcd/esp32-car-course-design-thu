#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "motor.h"

/* 三路电机的真实转速快照；rpm 为编码器测得的转速大小，不是 PWM 百分比。 */
typedef struct {
    float rpm[MOTOR_NUM];
    bool valid[MOTOR_NUM];
} encoder_rpm_t;

/* 同一次采样的原始上升沿数；用于横移轮速比例/停转诊断，不使用显示低通。 */
typedef struct {
    uint32_t sequence;
    uint32_t sample_ms;
    uint32_t interval_ms;
    int32_t pulses[MOTOR_NUM];
} encoder_motion_t;

encoder_motion_t encoder_get_motion(void);

/** 初始化三路 PCNT 脉冲计数器。初始化后不驱动电机。 */
void encoder_init(void);

/**
 * 读取并更新三路 RPM。
 * 建议每 200~300ms 调用一次；第一次调用仅建立基准，valid 为 false。
 */
void encoder_update(void);

/** 获取最近一次 RPM 快照。 */
encoder_rpm_t encoder_get_rpm(void);

/*
 * 里程计：编码器累计脉冲（单相计数无方向，正反转都累加）。
 * 由 encoder_update() 采样时累计——调用方需周期调用 encoder_update()
 * （内部 200ms 限速，并用互斥锁串行化；TFT/行走/球任务可同时调用）。
 * 未标定 PPR 也可用：只用于"前进多少脉冲就退回多少脉冲"的相对里程。
 */
/** 里程计清零（进入某段行程时调用）。 */
void encoder_odom_reset(void);
/** 读某路累计脉冲（方向无关，恒非负）。 */
int32_t encoder_odom_get(int motor);
