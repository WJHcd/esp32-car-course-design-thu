#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 云台控制接口（MG90S × 2：pan 左右 + tilt 上下）
 *
 * MG90S 为普通 PWM 舵机（50Hz，脉宽 0.5~2.5ms 对应 0~180°）。
 * 无硬件角度反馈 → 本模块在软件中记录"指令角度"。
 *
 * ⚠️ 引脚当前未定（GPIO 紧张），先定义接口与软件角度记录；
 *    引脚由 gimbal_init(pan_pin, tilt_pin) 传入。
 *
 * ============================================================
 * 云台机械限位（2026-08-26 硬件确认）：
 *   - pan（水平/左右）：-120° ~ +120°（0°=正前方，负=左，正=右）
 *   - tilt（垂直/上下）：-30°（向下）~ +90°（向上），0°=水平
 *   - 两轴极限处均有物理限位
 *
 * 角度约定：0° 为机械零位（pan 正前方 / tilt 水平）。
 *   pan  ：负=向左转，正=向右转
 *   tilt ：负=向下俯，正=向上仰
 * ============================================================ */

/* 云台机械限位（宏，供其他模块使用） */
#define GIMBAL_PAN_LIMIT_MIN    (-120)
#define GIMBAL_PAN_LIMIT_MAX    ( 120)
#define GIMBAL_TILT_LIMIT_MIN   ( -30)
#define GIMBAL_TILT_LIMIT_MAX   (  90)

/**
 * @brief 初始化云台
 * @param pan_pin  左右舵机 PWM GPIO（0=暂不接线）
 * @param tilt_pin 上下舵机 PWM GPIO（0=暂不接线）
 *
 * 会执行一次上电自动校准：pan/tilt 先向机械限位方向转，
 * 直到物理限位停止（堵转），以此确定机械零位，再回到 0°。
 * ⚠️ 校准期间舵机会有堵转电流，接线后需确认供电能力。
 */
void gimbal_init(int pan_pin, int tilt_pin);

/**
 * @brief 左右转至目标角度（限位内自动夹紧），并记录
 * @return 实际生效角度（可能被限位截断）
 */
int gimbal_set_pan(int angle_deg);

/**
 * @brief 上下转至目标角度（限位内自动夹紧），并记录
 * @return 实际生效角度（可能被限位截断）
 */
int gimbal_set_tilt(int angle_deg);

/**
 * @brief 读取记录的左右角度（软件记忆，非硬件反馈）
 */
int gimbal_get_pan(void);

/**
 * @brief 读取记录的上下角度（软件记忆，非硬件反馈）
 */
int gimbal_get_tilt(void);

/**
 * @brief 是否已完成上电校准
 */
bool gimbal_is_calibrated(void);

/**
 * @brief 舵机方向测试（独立测试入口，用于提前标定安装方向）
 *
 * 依次让 pan/tilt 以原始角度 0°→180°→0° 摆动（脉宽 0.5ms→2.5ms），
 * 串口提示观察每个舵机的物理转向（左/右、上/下）。
 * 结果用于确定"物理角度 ↔ 舵机脉宽"的符号映射。
 */
void gimbal_axis_direction_test(void);
