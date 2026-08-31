#include "ascii_preview.h"
#include <stdio.h>
#include "esp_timer.h"
#include "camera_line.h"

/* 亮度→字符：判黑段固定 '#'（可视化检测阈值），其余按亮度渐变 */
static const char ART_CHARS[] = " .:-=+*";
#define ART_LEVELS 7

/* 整帧一次 printf，避免与其他任务日志交错撕裂画面 */
static char art_buf[ASCII_PREVIEW_HEIGHT * (ASCII_PREVIEW_WIDTH + 1) + 1];
static int64_t s_last_print_us;

void ascii_preview_on_frame(const uint8_t *rgb888, int width, int height)
{
    const int64_t now = esp_timer_get_time();
    if (s_last_print_us != 0 &&
        now - s_last_print_us < (int64_t)ASCII_PREVIEW_INTERVAL_MS * 1000) {
        return;
    }
    s_last_print_us = now;

    /* cell 取整除：320/64=5、240/24=10；640x480 回退分辨率下同样成立 */
    const int cell_w = width / ASCII_PREVIEW_WIDTH;
    const int cell_h = height / ASCII_PREVIEW_HEIGHT;

    /* 扫描窗几何（与 detect_line_centroid 同一套宏），调参时直接对照字符画 */
    {
        const int win_w = width * CAM_SCAN_WIDTH_PCT / 100;
        const int xs = (width - win_w) / 2;
        const int xe = xs + win_w;
        const int band_half = height * CAM_SCAN_BAND_HEIGHT_PCT / 200;
        const int yc = height - height * CAM_SCAN_CENTER_FROM_BOTTOM_PCT / 100;
        int yt = yc - band_half; if (yt < 0) yt = 0;
        int yb = yc + band_half; if (yb > height) yb = height;
        printf("扫描窗: 字符列%d-%d 字符行%d-%d (共%d列%d行) 阈值=%d\n",
               xs / cell_w, (xe - 1) / cell_w,
               yt / cell_h, (yb - 1) / cell_h,
               ASCII_PREVIEW_WIDTH, ASCII_PREVIEW_HEIGHT, CAM_BIN_THRESHOLD);
    }

    char *p = art_buf;
    for (int cy = 0; cy < ASCII_PREVIEW_HEIGHT; cy++) {
        for (int cx = 0; cx < ASCII_PREVIEW_WIDTH; cx++) {
            /* 对每个字符格内的像素求平均亮度 */
            uint32_t lum_sum = 0;
            for (int dy = 0; dy < cell_h; dy++) {
                const uint8_t *px = rgb888
                    + (size_t)(cy * cell_h + dy) * width * 3
                    + (size_t)cx * cell_w * 3;
                for (int dx = 0; dx < cell_w; dx++) {
                    lum_sum += px[dx * 3] + px[dx * 3 + 1] + px[dx * 3 + 2];
                }
            }
            const uint8_t lum = (uint8_t)(lum_sum / (uint32_t)(cell_w * cell_h * 3));
            if (lum < CAM_BIN_THRESHOLD) {
                *p++ = '#';
            } else {
                const int idx = (uint32_t)(lum - CAM_BIN_THRESHOLD)
                                * (ART_LEVELS - 1) / (255 - CAM_BIN_THRESHOLD);
                *p++ = ART_CHARS[idx];
            }
        }
        *p++ = '\n';
    }
    *p = '\0';
    printf("%s", art_buf);
}
