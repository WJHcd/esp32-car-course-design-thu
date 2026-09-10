#pragma once

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
 * @brief GIF 表情动画帧数（资源无效返回 0）
 */
int display_gif_count(void);

/**
 * @brief 绘制 GIF 第 index 帧并推屏
 * @return 该帧延时 ms（资源无效返回 0）
 */
int display_gif_draw(int index);
