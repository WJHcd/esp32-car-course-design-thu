#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "infrared.h"

/* ============================================================
 * 行走策略：状态机（巡线 + 超声波避障）+ PID
 * 从参考 STM32 程序 Car_main.c 的 car_tim() 移植到 ESP32-S3。
 * 由 main.c 周期调用 walking_update() 驱动小车。
 * ============================================================ */

/* 状态机枚举（与参考程序 enum CarState 一致） */
typedef enum {
    CAR_STATE_RUN = 0,            /* 正常运行（巡线） */
    CAR_STATE_MEET_FACE,          /* 全黑 → 遇线/路口 */
    CAR_STATE_BACKWARD,           /* 后退 */
    CAR_STATE_SEARCH_EXIT_LEFT,   /* 向左寻找出口 */
    CAR_STATE_SEARCH_EXIT_RIGHT,  /* 向右寻找出口 */
    CAR_STATE_BACK_TO_FORWARD,    /* 回到前进方向 */
    CAR_STATE_DASH,               /* 冲刺 */
    CAR_STATE_AVOIDING,           /* 避障（超声波触发） */
    CAR_STATE_ROTATE,             /* 旋转回位 */
} car_state_t;

/**
 * @brief 初始化行走策略（含底层电机驱动 motor_init）
 */
void walking_init(void);

/**
 * @brief 周期调用（建议 20ms），根据传感器数据执行状态机并驱动电机
 * @param pat          四路循迹模式（a/b/c/d，0=在线，-1=离线）
 * @param distance_cm  超声波距离（ULTRASONIC_INVALID = 无效）
 */
void walking_update(const infrared_pattern_t *pat, uint16_t distance_cm);

/**
 * @brief 获取当前状态（测试/显示用）
 */
car_state_t walking_get_state(void);

/**
 * @brief 开关电机输出
 * @param en  true=驱动电机（正常跑车）；false=只跑状态机不驱动（安全自检）
 */
void walking_set_output_enabled(bool en);

/**
 * @brief 状态枚举 → 名称字符串（打印用）
 */
const char *walking_state_name(car_state_t s);
