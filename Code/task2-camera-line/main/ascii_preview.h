#pragma once
#include <stdint.h>

/* ============================================================
 * 摄像头画面 ASCII 串口预览（不依赖 TFT 验证摄像头是否工作）
 *
 * 把解码后的 RGB888 帧降采样成 64x24 字符画，每秒打印到串口，
 * idf.py monitor 直接可看（80 列终端可完整显示）。
 * 映射规则：
 *   '#'      —— 低于 CAM_BIN_THRESHOLD 判为黑色的像素，
 *               与黑线检测算法同一阈值，即算法"看到"的黑线
 *   " .:-=+*" —— 其余像素按亮度映射，越亮字符越密集
 * ============================================================ */

#define ASCII_PREVIEW_WIDTH        64    /* 字符画宽（列数） */
#define ASCII_PREVIEW_HEIGHT       24    /* 字符画高（行数） */
#define ASCII_PREVIEW_INTERVAL_MS  1000  /* 打印间隔 */

/**
 * @brief 帧回调：降采样并按限频间隔输出 ASCII 画面
 *
 * 签名兼容 camera_line_frame_cb_t，可直接注册给
 * camera_line_set_frame_cb()；内部自带限频，每帧调用也只按
 * ASCII_PREVIEW_INTERVAL_MS 打印。
 * 注意：在图像处理任务上下文中执行（同帧回调约束）。
 */
void ascii_preview_on_frame(const uint8_t *rgb888, int width, int height);
