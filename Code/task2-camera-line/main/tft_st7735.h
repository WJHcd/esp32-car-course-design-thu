#pragma once

#include <stdint.h>

/** 初始化已接好的 LQ-TFT18SPIV33 SPI 面板，但不启动仪表刷新任务。 */
void tft_st7735_init(void);

/** 启动低优先级仪表刷新任务；须先调用 tft_st7735_init()。 */
void tft_st7735_start_telemetry(void);

/** 仅面板诊断：循环显示红、绿、蓝、黑，不依赖电机、编码器或传感器。 */
void tft_st7735_panel_test(void);

/** 更新供显示任务使用的最近一次有效超声距离；传 ULTRASONIC_INVALID 表示无效。 */
void tft_st7735_set_distance(uint16_t distance_cm);
