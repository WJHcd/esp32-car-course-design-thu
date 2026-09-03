#pragma once

#include <stdint.h>
#include <stdbool.h>

/** 初始化已接好的 LQ-TFT18SPIV33 SPI 面板，但不启动仪表刷新任务。 */
void tft_st7735_init(void);

/** 启动低优先级仪表刷新任务；须先调用 tft_st7735_init()。 */
void tft_st7735_start_telemetry(void);

/**
 * 切换 TFT 显示所有权。false=遥测页，true=摄像头页。
 * 摄像头页期间遥测任务暂停写屏，避免两条任务交叉发送 SPI 命令。
 */
void tft_st7735_set_camera_mode(bool enabled);

/** 仅面板诊断：循环显示红、绿、蓝、黑，不依赖电机、编码器或传感器。 */
void tft_st7735_panel_test(void);

/** 更新供显示任务使用的最近一次有效超声距离；传 ULTRASONIC_INVALID 表示无效。 */
void tft_st7735_set_distance(uint16_t distance_cm);

/**
 * 更新摄像头巡线状态（任务二）：屏幕第 5 行显示
 *   见线且稳定：E:+0.12 OK（误差，左负右正）
 *   见线未稳定：E:+0.12 !（连续帧数/姿态稳定等待还没满足）
 *   丢线时：    E:LOST
 * @param has_line  是否检测到黑线
 * @param error     黑线质心误差 [-1,+1]
 * @param stable    状态是否稳定（camera_line_result_t::stable）
 */
void tft_st7735_set_line(bool has_line, float error, bool stable);

/**
 * 将一帧 RGB888 图像显示到 TFT（摄像头画面预览用）。
 * 将完整输入帧等比例缩放到 128x160 内，空白区域填黑，确保位于
 * 摄像头两侧的球和识别框不会被中央裁剪掉。按该屏 R/B 物理互换
 * 做了通道修正。未初始化或不在摄像头模式时直接返回。
 * 注意：全屏刷新约需 0.33s（SPI 1MHz），调用方注意帧率控制。
 */
void tft_st7735_show_camera_frame(const uint8_t *rgb888, int frame_w, int frame_h);
