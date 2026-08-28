#pragma once
#include "driver/gpio.h"

/* =====================================================================
 * 任务二引脚分配（ESP32-S3-DevKitC-1）
 * 电机、编码器、超声波、TFT 沿用 task1_ver2.0；巡线改为 USB UVC 摄像头。
 *
 * 引脚占用现状：
 *   电机/编码器   GPIO 1,2,4,5,6,7,8,9,10,11,12,14,16
 *   TFT SPI        GPIO 13,15,17
 *   USB UVC Host   GPIO 19(D-) / GPIO20(D+)
 *   板上 WS2812   GPIO 38（已避开）
 *   UART0 console GPIO 43,44（已避开）
 *   超声波         GPIO 39,40
 *
 * ⚠️ 注意事项：
 *   GPIO0/3/45/46 为 strapping 脚（上电时序敏感），本任务已完全避开；
 *   GPIO19/20 为原生 USB 数据脚（USB_D-/USB_D+），本任务接 UVC 摄像头；
 *             下载/监视使用外接 USB-TTL 的 UART0（GPIO43/44）；
 *   GPIO26~37 为 flash/PSRAM，不可用；
 *   GPIO39~42 为 JTAG 脚（MTCK/MTDO/MTDI/MTMS），本任务不做 JTAG 调试，
 *             可当普通 GPIO 用；
 *   GPIO22~25 在 ESP32-S3 上不存在（编号空洞），GPIO47/48 未引出。
 * ===================================================================== */

/* ---- USB UVC 摄像头（JQ-CAM12-720P） ----
 * D- -> GPIO19，D+ -> GPIO20；由 USB Host 驱动直接控制，无需 GPIO API。 */
#define CAM_USB_DM_GPIO GPIO_NUM_19
#define CAM_USB_DP_GPIO GPIO_NUM_20

/* ---- HC-SR04 超声波 ---- */
#define US_TRIG_GPIO   GPIO_NUM_39
#define US_ECHO_GPIO   GPIO_NUM_40

/* ---- 三路电机测速编码器（仅使用 A 相：测速大小，方向由电机指令决定） ----
 * 已实物接线：A/B/C 三个电机的编码器 A 相分别接 GPIO12/14/16。
 * GPIO13/15/17 留给 TFT 的 SCK/SDI/DC，故不要再接编码器 B 相。 */
#define ENC_A_GPIO      GPIO_NUM_12
#define ENC_B_GPIO      GPIO_NUM_14
#define ENC_C_GPIO      GPIO_NUM_16

/* ---- LQ-TFT18SPIV33（1.8 寸 4 线 SPI，ILI9163） ----
 * 已实物接线：SCK=13, SDI(MOSI)=15, DC=17。
 * CS 固定接 GND（本项目 SPI 总线上只有本屏），RST 接 ESP32 EN，
 * 因此不占用额外 GPIO。 */
#define TFT_SCLK_GPIO   GPIO_NUM_13
#define TFT_MOSI_GPIO   GPIO_NUM_15  /* TFT SDI */
#define TFT_DC_GPIO     GPIO_NUM_17
