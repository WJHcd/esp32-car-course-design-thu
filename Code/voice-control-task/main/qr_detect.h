#pragma once
#include <stdbool.h>
#include <stdint.h>

/* ============================================================
 * 二维码识别接口（基于 espressif/quirc）
 *
 * 输入：一帧 8bit 灰度图
 * 输出：是否解出码 + 文本 + 码在画面中的位置
 *
 * ⚠️ 解码耗时几十~几百 ms，调用方必须节流（建议每 ~300ms 一次）。
 * ============================================================ */

#define QR_TEXT_MAX   32

typedef struct {
    bool   detected;        /* 本帧是否解出至少一个码 */
    char   text[QR_TEXT_MAX]; /* 解码文本（取第一个码，UTF-8/ASCII） */
    float  x_ratio;         /* 码中心 X [0,1]，0.5=画面正中 */
    float  y_ratio;         /* 码中心 Y [0,1]，0=顶部 */
    int    count;           /* 解出的码数量 */
} qr_result_t;

/**
 * @brief 初始化解码器（重复调用会重建 quirc 实例）
 * @param width   图像宽（像素）
 * @param height  图像高（像素）
 * @param max_codes 保留接口（当前实现忽略，扫描全部码）
 */
void qr_detect_init(int width, int height, int max_codes);

/**
 * @brief 对一帧灰度图执行解码
 * @param gray  灰度像素（width*height 字节，1=黑 255=白 或相反均可，flip 兜底）
 * @param out   输出结果（detected=false 时 text 为空）
 * @return detected
 */
bool qr_detect_process_gray(const uint8_t *gray, qr_result_t *out);
