#pragma once
#include "driver/gpio.h"

/* =====================================================================
 * 二维码识别表情+跳舞 · 引脚分配（ESP32-S3-DevKitC-1）
 *
 * 2026-09-08 与 WJHcd/esp32-car-course-design 仓库（task1_ver2.0 /
 * task2-camera-line，车上实物接线并已验证）对齐：
 *   - 电机 D24A 引脚两版一致，沿用；
 *   - TFT 按车上实际接线改为 SCK=13 / SDI=15 / DC=17；
 *     CS 硬件接地、RST 接 ESP32 的 EN，不再占用 GPIO
 *     （此前 21/18/42/41/40 与实车不符，是 TFT 白屏的直接原因）；
 *   - 面板实为 ILI9163（非 ST7735），初始化序列见 display.c。
 *
 * 引脚占用现状（与 task2-camera-line 一致）：
 *   电机 D24A       GPIO 1,2,4,5,6,7,8,9,10,11
 *   编码器（已接）  GPIO 12,14,16（本工程不使用）
 *   TFT SPI         GPIO 13(SCK) 15(SDI) 17(D/C)
 *   USB UVC 摄像头  GPIO 19(D-) / 20(D+)
 *   超声波 HC-SR04  GPIO 39(TRIG) 40(ECHO)
 *   超声波（已接）  GPIO 39,40（本工程不使用）
 *   UART0 console   GPIO 43,44
 *   避开：GPIO0/3/45/46(strapping)、26~37(flash/PSRAM)、38(WS2812)
 * ===================================================================== */

/* ---- 电机（D24A，同 task1_ver2.0 / motor-drive-test） ---- */
#define GPIO_STBY   GPIO_NUM_1   /* 全局使能，高有效 */
#define GPIO_PWMA   GPIO_NUM_2   /* LEDC 20kHz, 10bit */
#define GPIO_PWMB   GPIO_NUM_6
#define GPIO_PWMC   GPIO_NUM_9
#define GPIO_AIN1   GPIO_NUM_4
#define GPIO_AIN2   GPIO_NUM_5
#define GPIO_BIN1   GPIO_NUM_7
#define GPIO_BIN2   GPIO_NUM_8
#define GPIO_CIN1   GPIO_NUM_10
#define GPIO_CIN2   GPIO_NUM_11

/* ---- TFT 显示屏 LQ_TFT18SPIV33（1.8寸 128x160，ILI9163，7Pin） ----
 * 按车上实物接线（task1_ver2.0 / task2-camera-line 已验证）：
 *   SCK=13, SDI(MOSI)=15, DC=17；
 *   CS 固定接 GND（SPI 总线上只有本屏），RST 接 ESP32 的 EN，
 *   驱动侧不配置 CS/RST 引脚。 */
#define TFT_PIN_SCK   GPIO_NUM_13
#define TFT_PIN_SDI   GPIO_NUM_15   /* MOSI */
#define TFT_PIN_DC    GPIO_NUM_17

/* ---- HC-SR04 超声波（同 task1/2 接线，攻城槌撞击检测用） ---- */
#define US_TRIG_GPIO   GPIO_NUM_39
#define US_ECHO_GPIO   GPIO_NUM_40

/* ---- 摄像头 USB Host（GPIO19/20 由硬件 USB 外设使用，无需 GPIO 定义） ---- */
