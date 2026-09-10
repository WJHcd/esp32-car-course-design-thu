#pragma once
#include <stdint.h>

/* ============================================================
 * TFT 显示屏模块（LQ_TFT18SPIV33，1.8寸 128x160，ST7735，SPI）
 *
 * 用于显示"表情"（face_id）：
 *   0 = 笑脸  1 = 哭脸  2 = 惊讶  3 = 眨眼  4 = 爱心眼
 * display_idle() 显示"待机"表情（点眼 + 直线嘴）。
 * ============================================================ */

/**
 * @brief 初始化 SPI + ST7735 面板
 */
void display_init(void);

/**
 * @brief 显示指定表情
 * @param face_id  表情编号（0=笑脸 1=哭脸 2=惊讶 3=眨眼 4=爱心眼，见 display.c）
 */
void display_face(int face_id);

/**
 * @brief 待机画面（等待二维码时的表情）
 */
void display_idle(void);

/**
 * @brief 选择 GIF 动画资源（0=搜索表情 1=撞击表情），并重新解析
 */
void display_gif_use(int asset);

/**
 * @brief 整屏填充并推屏（状态页铺底/清屏）
 */
void display_fill(uint16_t color);

/**
 * @brief 在第 row 行（0..8）显示 ASCII 文本（小写自动转大写，最多 20 字符）
 */
void display_text(uint8_t row, const char *text, uint16_t color);

/**
 * @brief 当前资源的 GIF 帧数（资源无效返回 0）
 */
int display_gif_count(void);

/**
 * @brief 绘制 GIF 第 index 帧并推屏
 * @return 该帧延时 ms（资源无效返回 0）
 */
int display_gif_draw(int index);
