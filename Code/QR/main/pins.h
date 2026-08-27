#pragma once
#include "driver/gpio.h"

/* =====================================================================
 * 二维码识别表情+跳舞 · 引脚分配（ESP32-S3-DevKitC-1）
 *
 * 占用清单：
 *   摄像头 USB Host    GPIO19/20（原生 USB，下载/监视必须空出）
 *   电机（D24A）       GPIO1(STBY) 2/6/9(PWM) 4/5/7/8/10/11(方向)
 *   TFT 显示屏(7脚)    GPIO21(SCK) 18(SDI) 42(CS) 41(D/C) 40(RST)
 *   调试 UART0         GPIO43/44
 *   避开：GPIO0/3/45/46(strapping)、26~37(flash/PSRAM)、38(WS2812)
 * ===================================================================== */

/* ---- 电机（D24A，同 task1） ---- */
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

/* ---- TFT 显示屏 LQ_TFT18SPIV33（1.8寸 128x160，ST7735，7Pin） ----
 * 7Pin：SCK / SDI(MOSI) / CS / D/C / RST / VCC / GND（无 BL，背光常亮）
 * 注意：本模块无 MISO，SPI 只写。 */
#define TFT_PIN_SCK   GPIO_NUM_21
#define TFT_PIN_SDI   GPIO_NUM_18   /* MOSI */
#define TFT_PIN_CS    GPIO_NUM_42
#define TFT_PIN_DC    GPIO_NUM_41
#define TFT_PIN_RST   GPIO_NUM_40

/* ---- 摄像头 USB Host（GPIO19/20 由硬件 USB 外设使用，无需 GPIO 定义） ---- */
