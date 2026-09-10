#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* ============================================================
 * BLE 手机遥控协议/安全状态机（纯逻辑，无 BLE/电机依赖）
 *
 * 由 task3-ble-control 的 control.c 原样移植（djj）：
 *   - nRF Toolbox / 文本协议：ARM/X/S/F/B/L/R/Q/E/M,f,t,s/?
 *   - Adafruit Bluefruit Control Pad 按键包（!B..，含校验）
 * 控制输出三分量 f/t/s（前向/旋转/横移，单位 %），供上层调用
 * motion_drive() 驱动（混控与标定增益见 motion.c）。
 *
 * 安全设计（勿削弱）：
 *   - connected + armed 双闸门，未连接/未 ARM 一律停；
 *   - 文本点动 300ms 命令租约；按键模式 10s 硬上限
 *     （Bluefruit 只发按下/松开边沿，无心跳，防丢松开）；
 *   - ESTOP（按键 2 / "X"）断开 arming，需重新 ARM。
 * ============================================================ */

/* 分量幅度（%）。方向语义：f 前向正、t 旋转正=逆时针、s 横移正=左。 */
#define DRIVE_PCT 18
#define TURN_PCT 18
#define STRAFE_PCT 15
#define AXIS_MAX_PCT 25      /* 文本 M,f,t,s 三轴范围上限 */
#define COMMAND_LEASE_MS 300 /* 文本点动：无新命令 300ms 后停车 */
/* Bluefruit 只发边沿事件：给丢失的松开一个硬上限 */
#define BUTTON_HOLD_MAX_MS 10000

typedef struct {
    bool connected, armed;   /* 双闸门：已连接 && 已 ARM 才允许运动 */
    bool button_hold;        /* 当前处于按键按住模式（走 10s 上限） */
    uint8_t active_button;   /* 当前按住的按键号（1~8） */
    int f, t, s;             /* 输出三分量：前向/旋转/横移（%） */
    int64_t last_motion_ms;  /* 最近一次运动指令时间戳（ms） */
} control_t;

/**
 * @brief 连接状态变化（GAP 事件调用）：断开会清空全部运动并撤 ARM
 * @param c          状态机
 * @param connected  是否已连接
 */
void control_link(control_t *c, bool connected);

/**
 * @brief 处理一条命令（一次 write = 一条 ASCII 命令 或 一个 !B 按键包）
 * @param c    状态机
 * @param data 命令数据（无需以 0 结尾）
 * @param len  数据长度
 * @param now  当前毫秒时间戳（esp_timer_get_time()/1000）
 * @return 回复字符串（静态/调用方不释放；"OK"/"ARMED"/"HOLD"/"ERR xxx" 等）
 */
const char *control_command(control_t *c, const char *data, size_t len, int64_t now);

/**
 * @brief 周期调用（建议 20ms）：租约/按住超时检查
 * @return true=本次刚好超时停车（可打日志）
 */
bool control_tick(control_t *c, int64_t now);
