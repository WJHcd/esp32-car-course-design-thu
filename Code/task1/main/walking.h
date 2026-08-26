#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "infrared.h"

/* 固定功能只负责前进方向循迹和定时避障；三轮完整全向运动学后续单独标定。 */
typedef enum {
    CAR_STATE_RUN = 0,
    CAR_STATE_LINE_SEARCH,
    CAR_STATE_CORNER_SEARCH,
    CAR_STATE_AVOID_STOP,
    CAR_STATE_AVOID_BACKUP,
    CAR_STATE_AVOID_TURN_OUT,
    CAR_STATE_AVOID_BYPASS,
    CAR_STATE_AVOID_TURN_BACK,
    CAR_STATE_AVOID_FIND_LINE,
    CAR_STATE_FAILSAFE_STOP,
} car_state_t;

typedef struct {
    int forward_percent;  /* 正=沿车头方向前进，负=后退 */
    int turn_percent;     /* 正负方向由 walking.c 的 STEERING_SIGN 标定 */
} walking_command_t;

/** 初始化状态机和底层电机；初始化后电机保持禁用。 */
void walking_init(void);

/**
 * 周期调用行走控制器。
 *
 * @param pat          从左到右的四路红外数据（1=黑线，0=白底）
 * @param distance_cm  本周期新得到的超声距离；没有新样本或测量失败时传
 *                     ULTRASONIC_INVALID。不要在多个周期重复传同一个旧值。
 * @param now_ms       单调递增的毫秒时间（例如 esp_timer_get_time()/1000）
 */
void walking_update(const infrared_pattern_t *pat,
                    uint16_t distance_cm,
                    uint32_t now_ms);

/** 开关真实电机输出；关闭时会刹车并拉低 STBY。 */
void walking_set_output_enabled(bool enabled);

/** 从故障停车状态恢复到 RUN，同时清空丢线和障碍确认计数。 */
void walking_reset(uint32_t now_ms);

car_state_t walking_get_state(void);
const char *walking_state_name(car_state_t state);
walking_command_t walking_get_last_command(void);
float walking_get_last_error(void);
