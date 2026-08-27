#pragma once
#include "driver/gpio.h"

/* =====================================================================
 * 任务一 引脚分配（ESP32-S3-DevKitC-1）
 * 电机引脚沿用 motor-drive-test（已在车上接好）：见 motor.c
 * 本文件只定义"循迹 + 超声波"新增传感器的引脚，方便统一调整。
 *
 * 引脚占用现状：
 *   电机/编码器   GPIO 1,2,4,5,6,7,8,9,10,11,12,13,14,15,16,17
 *   板上 WS2812   GPIO 38（已避开）
 *   原生 USB      GPIO 19,20（USB_D-/USB_D+，下载/监视必须空出）
 *   UART0 console GPIO 43,44（已避开）
 *   留给本任务     GPIO 18,21,39,40,41,42（见下）
 *
 * ⚠️ 注意事项：
 *   GPIO0/3/45/46 为 strapping 脚（上电时序敏感），本任务已完全避开；
 *   GPIO19/20 为原生 USB 数据脚（USB_D-/USB_D+），必须空出给下载/监视；
 *   GPIO26~37 为 flash/PSRAM，不可用；
 *   GPIO39~42 为 JTAG 脚（MTCK/MTDO/MTDI/MTMS），本任务不做 JTAG 调试，
 *             可当普通 GPIO 用；
 *   GPIO22~25 在 ESP32-S3 上不存在（编号空洞），GPIO47/48 未引出。
 * ===================================================================== */

/* ---- 四路红外循迹 LQ-R4CHV（数字量输出，OUT1~OUT4） ---- */
/* a/b/c/d 映射（2026-08-25 修正）：a=OUT4, b=OUT3, c=OUT2, d=OUT1 */
#define IR_GPIO_OUT1   GPIO_NUM_18   /* → d */
#define IR_GPIO_OUT2   GPIO_NUM_21   /* → c */
#define IR_GPIO_OUT3   GPIO_NUM_41   /* → b */
#define IR_GPIO_OUT4   GPIO_NUM_42   /* → a */

/* 黑线对应的 GPIO 电平（2026-08-25 实测修正）：
 *   本模块为常规极性：黑线=低电平、白底=高电平，故 IR_LINE_LEVEL=0。
 * 若实测相反，把 0 改成 1 即可（测试程序 test_infrared 里可直接观察）。 */
#define IR_LINE_LEVEL  0

/* ---- HC-SR04 超声波 ---- */
#define US_TRIG_GPIO   GPIO_NUM_39
#define US_ECHO_GPIO   GPIO_NUM_40
