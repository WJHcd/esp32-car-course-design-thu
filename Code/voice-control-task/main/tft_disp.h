#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * TFT 状态屏驱动（独立裁剪版）
 *
 * 面板：LQ-TFT18SPIV33，1.8" 128x160，ILI9163 控制器
 * 接线（task2 原样保留）：SCK=GPIO13 MOSI=GPIO15 DC=GPIO17
 *   CS 硬件接地、RST 接 EN（不占 GPIO）
 * 初始化序列/颜色修正照搬 task2 实机验证参数（2026-09-03）：
 *   MADCTL BGR 位 + 软件 R/B 交换，屏幕颜色语义正确。
 *
 * 字库：经典 5x7（见 tft_font_data.h，自动生成），仅大写字母
 * 数字与少量符号；文本按行（0~9 行，行距 16px）绘制。
 * ============================================================ */

/* 颜色宏（RGB565，已做该屏 R/B 物理互换修正：宏语义=屏幕所见） */
#define TFT_COLOR_BLACK   0x0000
#define TFT_COLOR_WHITE   0xFFFF
#define TFT_COLOR_RED     0x001F
#define TFT_COLOR_GREEN   0x07E0
#define TFT_COLOR_BLUE    0xF800
#define TFT_COLOR_YELLOW  0xFFE0

#define TFT_DISP_WIDTH    128
#define TFT_DISP_HEIGHT   160
#define TFT_DISP_ROWS     9       /* 文本行数（y=8+16*row，最后一行 y=136） */
#define TFT_DISP_TEXT_MAX  20     /* 每行最多字符（6px 步进） */

/**
 * @brief 初始化 TFT（SPI2 + DC，含面板初始化序列；约 0.4s 延时）
 *        幂等：重复调用直接返回。
 */
void tft_disp_init(void);

/**
 * @brief 整屏填充颜色（清屏用）
 */
void tft_disp_fill(uint16_t color);

/**
 * @brief 在第 row 行显示文本（自动换行截断到行宽）
 * @param row    0..TFT_DISP_ROWS-1
 * @param text   ASCII 文本（小写字母自动转大写；未知字符显示空格）
 * @param color  文本颜色（TFT_COLOR_*）
 */
void tft_disp_text(uint8_t row, const char *text, uint16_t color);

/** @brief SPI/面板是否已就绪 */
bool tft_disp_ready(void);
