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
 *   USB/UART0     GPIO 19,20 / 43,44（已避开）
 *   留给本任务     GPIO 0,3,18,21,39,40（见下）
 *
 * ⚠️ 注意事项：
 *   GPIO0 = 板上 BOOT 按键（内部上拉），只做输入（IR OUT2），不要接输出；
 *   GPIO3 = strapping 脚，做输入安全；GPIO39/40 = 芯片 JTAG 脚，
 *   默认 ESP-IDF 不占用，可当普通 GPIO 用（若以后开 JTAG 调试需避开）。
 * ===================================================================== */

/* ---- 四路红外循迹 LQ-R4CHV（数字量输出，OUT1~OUT4） ---- */
#define IR_GPIO_OUT1   GPIO_NUM_3    /* 对应参考程序 sensor1 → d */
#define IR_GPIO_OUT2   GPIO_NUM_0    /* 对应参考程序 sensor2 → a */
#define IR_GPIO_OUT3   GPIO_NUM_21   /* 对应参考程序 sensor3 → b */
#define IR_GPIO_OUT4   GPIO_NUM_39   /* 对应参考程序 sensor4 → c */

/* 黑线对应的 GPIO 电平：LQ-R4CHV（LM393 比较器）通常"检测到黑线=低电平"。
 * 若实测相反，把 0 改成 1 即可（测试程序 test_infrared 里可直接观察）。 */
#define IR_LINE_LEVEL  0

/* ---- HC-SR04 超声波 ---- */
#define US_TRIG_GPIO   GPIO_NUM_18
#define US_ECHO_GPIO   GPIO_NUM_40
