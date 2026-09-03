#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 云台俯仰舵机（MG90S，50Hz PWM，LEDC 实现）
 *
 * 移植自 camera-task 的 gimbal 接口，本工程只使用 tilt（俯仰）一轴：
 * 找球任务开始前把摄像头抬起获得俯视全场视角。
 *
 * 角度约定：0° = 水平（机械安装位），正 = 向上抬，负 = 向下压。
 * 限位见 GIMBAL_TILT_LIMIT_*（现场按实际安装调整）。
 *
 * ⚠️ 舵机需独立 5V 供电（与 ESP32 共地）；MG90S 堵转电流较大，
 *    不要从开发板 5V 引脚直供。
 * ============================================================ */

#define GIMBAL_TILT_LIMIT_MIN   ( -30)   /* 向下压极限 */
#define GIMBAL_TILT_LIMIT_MAX   (  90)   /* 向上抬极限 */

/* 舵机脉宽-角度映射（MG90S：0.5~2.5ms ↔ 0~180° 原始角度） */
#define GIMBAL_SERVO_FREQ_HZ    50
#define GIMBAL_SERVE_MIN_US     500      /* 原始 0° 脉宽 */
#define GIMBAL_SERVE_MAX_US     2500     /* 原始 180° 脉宽 */

/* 物理角度 ↔ 原始舵机角度的方向与零位映射（现场标定）：
 *   原始角 = GIMBAL_TILT_SIGN * 物理角 + GIMBAL_TILT_OFFSET
 * 若上电后"抬摄像头"实际变成下压，把 SIGN 改成 -1。 */
#define GIMBAL_TILT_SIGN        (-1)  /* 现场标定 2026-09-01：脉宽增大=向下，抬起需减小 */
#define GIMBAL_TILT_OFFSET      (90)     /* 物理 0°(水平) 对应原始 90°(中位) */

/**
 * @brief 初始化俯仰舵机（LEDC 50Hz 输出），并回水平位
 */
void gimbal_init(void);

/**
 * @brief 俯仰转至目标角度（限位内自动夹紧），并记录
 * @return 实际生效角度
 */
int gimbal_set_tilt(int angle_deg);

/**
 * @brief 读取记录的俯仰角度（软件记忆，非硬件反馈）
 */
int gimbal_get_tilt(void);
