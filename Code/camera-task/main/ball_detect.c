#include "ball_detect.h"
#include <string.h>

/* ============================================================
 * 小球颜色识别实现
 *
 * 算法（简单、可现场标定，适合 320x240 低分辨率实时处理）：
 *   1. 逐像素分类：红色（R 显著高于 G/B）/ 白色（三通道都高且接近）
 *   2. 分别统计红/白两类像素的质心（球心）、像素数、包围盒
 *   3. 取像素数较多的一类作为"球"（若画面中只应有一个球）
 *   4. 滤除：像素数过少（噪声）、面积占比过大（全屏误判）
 *
 * 每像素仅几次比较运算，320x240（76.8k 像素）在 240MHz 下 <1ms。
 * ============================================================ */

static int s_width;
static int s_height;

void ball_detect_init(int width, int height)
{
    s_width = width;
    s_height = height;
}

/* 判断像素是否为"红色球"颜色 */
static inline bool pixel_is_red(uint8_t r, uint8_t g, uint8_t b)
{
    return (r > BALL_RED_R_MIN) &&
           ((int)r - g > BALL_RED_RG_DELTA) &&
           ((int)r - b > BALL_RED_RB_DELTA);
}

/* 判断像素是否为"白色球"颜色（接近白：三通道都高且相差小） */
static inline bool pixel_is_white(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t maxc = r, minc = r;
    if (g > maxc) maxc = g;
    if (b > maxc) maxc = b;
    if (g < minc) minc = g;
    if (b < minc) minc = b;
    return (minc > BALL_WHITE_MIN) && (maxc - minc <= BALL_WHITE_TOL);
}

/* 单色统计：计算质心、像素数、包围盒 */
typedef struct {
    uint32_t count;      /* 匹配像素数 */
    uint64_t sum_x;      /* X 坐标累加 */
    uint64_t sum_y;      /* Y 坐标累加 */
    int min_x, min_y;    /* 包围盒 */
    int max_x, max_y;
    bool has_pixel;
} color_stat_t;

static void stat_init(color_stat_t *st)
{
    st->count = 0;
    st->sum_x = 0;
    st->sum_y = 0;
    st->has_pixel = false;
}

static void stat_add(color_stat_t *st, int x, int y)
{
    if (!st->has_pixel) {
        st->min_x = st->max_x = x;
        st->min_y = st->max_y = y;
        st->has_pixel = true;
    } else {
        if (x < st->min_x) st->min_x = x;
        if (x > st->max_x) st->max_x = x;
        if (y < st->min_y) st->min_y = y;
        if (y > st->max_y) st->max_y = y;
    }
    st->count++;
    st->sum_x += x;
    st->sum_y += y;
}

/* 把统计结果填入输出结构 */
static void stat_to_result(const color_stat_t *st, ball_color_t color,
                           ball_detect_result_t *out)
{
    out->detected = true;
    out->color = color;
    out->x_ratio = (float)st->sum_x / (st->count * s_width);
    out->y_ratio = (float)st->sum_y / (st->count * s_height);
    out->pixel_count = st->count;
    out->width_px = (uint16_t)(st->max_x - st->min_x + 1);
    out->height_px = (uint16_t)(st->max_y - st->min_y + 1);
    out->area_ratio = (float)st->count / (s_width * s_height);
}

void ball_detect_process_rgb888(const uint8_t *rgb, ball_detect_result_t *out)
{
    color_stat_t red, white;
    stat_init(&red);
    stat_init(&white);

    const uint8_t *p = rgb;
    for (int y = 0; y < s_height; y++) {
        for (int x = 0; x < s_width; x++) {
            uint8_t r = p[0], g = p[1], b = p[2];
            if (pixel_is_red(r, g, b)) {
                stat_add(&red, x, y);
            } else if (pixel_is_white(r, g, b)) {
                stat_add(&white, x, y);
            }
            p += 3;
        }
    }

    /* 面积占比上限（滤除全屏/大块误判） */
    uint32_t max_pixels = (uint32_t)(s_width * s_height * BALL_MAX_AREA_RATIO);

    bool red_ok = (red.count >= BALL_MIN_PIXELS) && (red.count <= max_pixels);
    bool white_ok = (white.count >= BALL_MIN_PIXELS) && (white.count <= max_pixels);

    if (red_ok && white_ok) {
        /* 两者都出现：取像素数多者（画面中只应有一个球） */
        if (red.count >= white.count) {
            stat_to_result(&red, BALL_COLOR_RED, out);
        } else {
            stat_to_result(&white, BALL_COLOR_WHITE, out);
        }
    } else if (red_ok) {
        stat_to_result(&red, BALL_COLOR_RED, out);
    } else if (white_ok) {
        stat_to_result(&white, BALL_COLOR_WHITE, out);
    } else {
        memset(out, 0, sizeof(*out));
        out->detected = false;
        out->color = BALL_COLOR_UNKNOWN;
    }
}

void ball_detect_get_thresholds(uint8_t *red_r_min, uint8_t *red_rg_delta,
                                uint8_t *red_rb_delta, uint8_t *white_min,
                                uint8_t *white_tol, uint32_t *min_pixels)
{
    if (red_r_min)    *red_r_min = BALL_RED_R_MIN;
    if (red_rg_delta) *red_rg_delta = BALL_RED_RG_DELTA;
    if (red_rb_delta) *red_rb_delta = BALL_RED_RB_DELTA;
    if (white_min)    *white_min = BALL_WHITE_MIN;
    if (white_tol)    *white_tol = BALL_WHITE_TOL;
    if (min_pixels)   *min_pixels = BALL_MIN_PIXELS;
}
