#pragma once
#include <stdbool.h>
#include "esp_err.h"

/* ============================================================
 * 蓝牙遥控功能（已实现：ESP32 BLE + NUS，兼容手机 App）
 *
 * 由 task3-ble-control 工程移植（djj）：
 *   - NimBLE 主机，外设角色，广播名 **MyCar-BLE**；
 *   - GATT 服务 Nordic UART Service (NUS)：RX 写入命令、
 *     TX 通知回复（连接后 App 订阅即可见状态/应答）；
 *   - 支持两类客户端：
 *       ① Adafruit Bluefruit Connect → Control Pad 方向键
 *          （1=使能/ARM，2=急停，↑↓←→=前进/后退/左右转，
 *           3/4=左右平移，按键 10s 硬上限防丢松开）；
 *       ② nRF Toolbox / nRF Connect 文本命令
 *          （ARM/X/S/F/B/L/R/Q/E/M,f,t,s/?，300ms 命令租约）。
 *
 * 安全：
 *   - 只有进入蓝牙遥控模式才广播；退出模式主动断开连接；
 *   - connected + armed 双闸门，未使能(STBY)不驱动电机；
 *   - 断开/退出自动撤 ARM + 刹停 + 关闭电机使能。
 * ============================================================ */

/**
 * @brief 初始化蓝牙遥控模块：NimBLE 协议栈 + NUS 服务 + 控制环任务
 *
 * 初始化后不广播、不使能电机；等待 app_menu 进入蓝牙遥控模式。
 */
esp_err_t bt_remote_init(void);

/**
 * @brief 进入蓝牙遥控模式（app_menu 切到 APP_MODE_BT 时调用）
 *
 * 开始广播 MyCar-BLE 等待手机连接；连接后手机按 1(ARM) 才能驱动。
 */
esp_err_t bt_remote_enter(void);

/**
 * @brief 退出蓝牙遥控模式（说"选择功能"返回菜单时调用）
 *
 * 主动断开连接、停止广播、撤 ARM、刹停并关闭电机使能。
 */
void bt_remote_exit(void);

/** @brief 当前是否处于蓝牙遥控模式 */
bool bt_remote_is_active(void);

/**
 * @brief 读取连接/使能状态（状态屏用；非阻塞）
 * @param[out] connected 是否已连接手机
 * @param[out] armed     是否已 ARM（允许驱动）
 */
void bt_remote_link_state(bool *connected, bool *armed);
