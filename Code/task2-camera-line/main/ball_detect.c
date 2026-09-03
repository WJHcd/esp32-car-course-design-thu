#include "ball_detect.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 三色识别实现：逐像素分类 → 4× 降采样占用网格 → flood-fill
 * 连通域 → 形状过滤（宽高比/填充率）→ 每色取最大（红取前二）。
 *
 * 每帧成本：全帧像素分类 ~0.5ms + 三张 80x60 网格的 flood fill
 * （合计 <4800 格）≈ 0.1ms，远小于 JPEG 解码。
 * ============================================================ */

#define GRID_W_MAX 80
#define GRID_H_MAX 60

static int s_width;
static int s_height;
static int s_grid_w;
static int s_grid_h;

typedef struct {
    uint8_t occ[GRID_W_MAX * GRID_H_MAX];   /* 占用标记（1=该格该色命中） */
    int16_t stack[GRID_W_MAX * GRID_H_MAX]; /* flood fill 栈 */
} color_grid_t;

static color_grid_t s_grid_black;
static color_grid_t s_grid_blue;
static color_grid_t s_grid_red;
static char s_debug_ascii[(GRID_W_MAX / 2 + 2) * (GRID_H_MAX / 2 + 1)];
static int s_diag_min, s_diag_max, s_diag_avg;   /* 中心区灰度诊断 */
static uint32_t s_dbg_black_px, s_dbg_blue_px, s_dbg_red_px;

void ball_detect_init(int width, int height)
{
    s_width = width;
    s_height = height;
    s_grid_w = width / BALL_GRID_DIV;
    s_grid_h = height / BALL_GRID_DIV;
    if (s_grid_w > GRID_W_MAX) s_grid_w = GRID_W_MAX;
    if (s_grid_h > GRID_H_MAX) s_grid_h = GRID_H_MAX;
    if (s_grid_w < 1) s_grid_w = 1;
    if (s_grid_h < 1) s_grid_h = 1;
}

/* ---- 像素级颜色分类（蓝/红先判，避免深蓝被黑误收）---- */
static inline int pixel_class(uint8_t r, uint8_t g, uint8_t b)
{
    if (b > BALL_BLUE_MIN_B && b - (r > g ? r : g) > BALL_BLUE_DELTA) {
        return 1;                                   /* 蓝 */
    }
    if (r > BALL_RED_R_MIN && r - g > BALL_RED_DELTA && r - b > BALL_RED_DELTA) {
        return 2;                                   /* 红 */
    }
    const uint16_t gray = (uint16_t)((r * 77 + g * 150 + b * 29) >> 8);
    if (gray < BALL_BLACK_GRAY_MAX) {
        return 3;                                   /* 黑 */
    }
    return 0;
}

/* ---- 形态学腐蚀（8 邻域，邻居 <5 个的格子剔除）：细长线段/边界被
 * 侵蚀消失，实心球团保留核心——空间上切断"球与线粘连"的连通域 ---- */
static void erode_grid(color_grid_t *g)
{
    static uint8_t tmp[GRID_W_MAX * GRID_H_MAX];
    memcpy(tmp, g->occ, sizeof(tmp));
    for (int y = 0; y < s_grid_h; y++) {
        for (int x = 0; x < s_grid_w; x++) {
            const int idx = y * s_grid_w + x;
            if (!tmp[idx]) continue;
            int nb = 0;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    const int nx = x + dx, ny = y + dy;
                    if (nx < 0 || nx >= s_grid_w || ny < 0 || ny >= s_grid_h) continue;
                    nb += tmp[ny * s_grid_w + nx];
                }
            }
            if (nb < 5) g->occ[idx] = 0;   /* 孤立/边缘格子剔除 */
        }
    }
}

/* ---- 单色连通域提取：从占用网格里 flood fill 出所有合格团块 ---- */
typedef struct {
    int count;          /* 格子数 */
    int sum_x, sum_y;   /* 格子坐标和（求质心） */
    int min_x, min_y, max_x, max_y;
} blob_cells_t;

/* 诊断宽松模式（2026-09-01）：跳过形状过滤，任何 >=MIN_CELLS 的连通域
 * 都接受（只保留最大的）——用于确认"球能否被框上"，框出来后再收紧。
 * 置 0 恢复严格形状过滤。 */
#define BALL_DIAG_LOOSE       1

static void extract_blobs(color_grid_t *g, ball_blob_t *out_blobs,
                          int max_out, int *out_count, const char *tag)
{
    *out_count = 0;
    memset(g->stack, 0, sizeof(g->stack));
    erode_grid(g);

    for (int y0 = 0; y0 < s_grid_h; y0++) {
        for (int x0 = 0; x0 < s_grid_w; x0++) {
            const int idx = y0 * s_grid_w + x0;
            if (!g->occ[idx]) continue;

            /* flood fill：从该格吞掉整个连通域（吞掉的格子清零防重复） */
            blob_cells_t c = {0};
            int top = 0;
            g->stack[top++] = idx;
            g->occ[idx] = 0;
            c.min_x = c.max_x = x0;
            c.min_y = c.max_y = y0;
            while (top > 0) {
                const int cur = g->stack[--top];
                const int cx = cur % s_grid_w;
                const int cy = cur / s_grid_w;
                c.count++;
                c.sum_x += cx;
                c.sum_y += cy;
                if (cx < c.min_x) c.min_x = cx;
                if (cx > c.max_x) c.max_x = cx;
                if (cy < c.min_y) c.min_y = cy;
                if (cy > c.max_y) c.max_y = cy;

                static const int nb[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
                for (int k = 0; k < 4; k++) {
                    const int nx = cx + nb[k][0];
                    const int ny = cy + nb[k][1];
                    if (nx < 0 || nx >= s_grid_w || ny < 0 || ny >= s_grid_h) continue;
                    const int nidx = ny * s_grid_w + nx;
                    if (g->occ[nidx]) {
                        g->occ[nidx] = 0;
                        g->stack[top++] = nidx;
                    }
                }
            }

            /* 形状过滤：紧凑球团 vs 细长线段（诊断宽松模式下跳过） */
            if (c.count < BALL_MIN_CELLS) continue;
            const int bw = c.max_x - c.min_x + 1;
            const int bh = c.max_y - c.min_y + 1;
            const float aspect = (float)(bw > bh ? bw : bh) / (bw < bh ? bw : bh);
            const float fill = (float)c.count / (float)(bw * bh);
            if (!BALL_DIAG_LOOSE) {
                if (aspect < BALL_ASPECT_MIN || aspect > BALL_ASPECT_MAX) continue;
                if (fill < BALL_FILL_MIN) continue;
            } else if (*out_count == 0 && (c.count > 6)) {
                /* 宽松诊断：打印首个大连通域的形状指标 */
                printf("[blob] %s: cells=%d bbox=%dx%d aspect=%.2f fill=%.2f\n",
                       tag, c.count, bw, bh, (double)aspect, (double)fill);
            }

            if (*out_count >= max_out) {
                /* 已满：替换最小的那个（保持最大的前 max_out 个） */
                int smallest = 0;
                for (int i = 1; i < max_out; i++) {
                    if (out_blobs[i].cell_count < out_blobs[smallest].cell_count) {
                        smallest = i;
                    }
                }
                if (c.count <= (int)out_blobs[smallest].cell_count) continue;
                *out_count = smallest;   /* 覆盖最小者 */
            }

            ball_blob_t *out = &out_blobs[*out_count];
            memset(out, 0, sizeof(*out));
            out->detected = true;
            out->cell_count = (uint32_t)c.count;
            out->x_ratio = ((float)c.sum_x / c.count + 0.5f) * BALL_GRID_DIV / (float)s_width;
            out->y_ratio = ((float)c.sum_y / c.count + 0.5f) * BALL_GRID_DIV / (float)s_height;
            out->width_px = (uint16_t)(bw * BALL_GRID_DIV);
            out->height_px = (uint16_t)(bh * BALL_GRID_DIV);
            out->x_min_px = (uint16_t)(c.min_x * BALL_GRID_DIV);
            out->y_min_px = (uint16_t)(c.min_y * BALL_GRID_DIV);
            out->area_ratio = (float)c.count * BALL_GRID_DIV * BALL_GRID_DIV /
                              (float)(s_width * s_height);
            (*out_count)++;
        }
    }

    /* 按面积从大到小排序（红角标两个时，[0] 恒为较大者） */
    for (int i = 1; i < *out_count; i++) {
        for (int j = i; j > 0; j--) {
            if (out_blobs[j].cell_count > out_blobs[j - 1].cell_count) {
                const ball_blob_t tmp = out_blobs[j];
                out_blobs[j] = out_blobs[j - 1];
                out_blobs[j - 1] = tmp;
            }
        }
    }
}

void ball_detect_process_rgb888(const uint8_t *rgb, ball_field_result_t *out)
{
    memset(&s_grid_black, 0, sizeof(s_grid_black));
    memset(&s_grid_blue, 0, sizeof(s_grid_blue));
    memset(&s_grid_red, 0, sizeof(s_grid_red));

    /* 逐像素分类并落到降采样网格：格内命中过半才算占用 */
    const int cells_x = s_grid_w, cells_y = s_grid_h;
    const int half = BALL_GRID_DIV * BALL_GRID_DIV / 2;
    uint32_t tot_black = 0, tot_blue = 0, tot_red = 0;
    static uint32_t gray_sum[GRID_W_MAX * GRID_H_MAX];
    static uint16_t gray_n[GRID_W_MAX * GRID_H_MAX];
    memset(gray_sum, 0, sizeof(gray_sum));
    memset(gray_n, 0, sizeof(gray_n));

    for (int gy = 0; gy < cells_y; gy++) {
        for (int py = 0; py < BALL_GRID_DIV; py++) {
            const int y = gy * BALL_GRID_DIV + py;
            if (y >= s_height) break;
            const uint8_t *row = rgb + (size_t)y * s_width * 3;
            for (int gx = 0; gx < cells_x; gx++) {
                int black = 0, blue = 0, red = 0;
                for (int px = 0; px < BALL_GRID_DIV; px++) {
                    const int x = gx * BALL_GRID_DIV + px;
                    if (x >= s_width) break;
                    const uint8_t *p = row + (size_t)x * 3;
                    const int g = (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8;
                    const int cidx = gy * cells_x + gx;
                    gray_sum[cidx] += (uint32_t)g;
                    gray_n[cidx]++;
                    switch (pixel_class(p[0], p[1], p[2])) {
                    case 1: blue++; tot_blue++; break;
                    case 2: red++; tot_red++; break;
                    case 3: black++; tot_black++; break;
                    default: break;
                    }
                }
                if (black >= half) s_grid_black.occ[gy * cells_x + gx] = 1;
                if (blue  >= half) s_grid_blue.occ[gy * cells_x + gx] = 1;
                if (red   >= half) s_grid_red.occ[gy * cells_x + gx] = 1;
            }
        }
    }
    s_dbg_black_px = tot_black;
    s_dbg_blue_px = tot_blue;
    s_dbg_red_px = tot_red;

    /* 调试视图：灰度数字地图（每 2 格取 1 格），字符 = 平均灰度/26：
     * '0' 最黑 … '4' 中灰 … '7' 白底，一眼看出球和黑线的位置与灰度 */
    {
        char *p = s_debug_ascii;
        for (int gy = 0; gy < cells_y; gy += 2) {
            for (int gx = 0; gx < cells_x; gx += 2) {
                const int idx = gy * cells_x + gx;
                const uint16_t n = gray_n[idx];
                const int avg = n ? (int)(gray_sum[idx] / n) : 255;
                int digit = avg / 26;
                if (digit > 9) digit = 9;
                *p++ = (char)('0' + digit);
            }
            *p++ = '\n';
        }
        *p = '\0';
    }

    /* 亮度诊断：中心 1/3 区域的灰度统计（定 BLACK_GRAY_MAX 用） */
    {
        uint32_t sum = 0;
        int n = 0;
        int mn = 255, mx = 0;
        const int x0 = s_width / 3, x1 = s_width * 2 / 3;
        const int y0 = s_height / 3, y1 = s_height * 2 / 3;
        for (int y = y0; y < y1; y += 4) {
            const uint8_t *row = rgb + (size_t)y * s_width * 3;
            for (int x = x0; x < x1; x += 4) {
                const uint8_t *p = row + (size_t)x * 3;
                const int g = (p[0] * 77 + p[1] * 150 + p[2] * 29) >> 8;
                sum += g;
                n++;
                if (g < mn) mn = g;
                if (g > mx) mx = g;
            }
        }
        if (n > 0) {
            s_diag_min = mn;
            s_diag_max = mx;
            s_diag_avg = (int)(sum / n);
        }
    }

    memset(out, 0, sizeof(*out));
    int n;
    extract_blobs(&s_grid_black, &out->black_ball, 1, &n, "黑");
    extract_blobs(&s_grid_blue, &out->blue_ball, 1, &n, "蓝");
    int red_n = 0;
    extract_blobs(&s_grid_red, out->red_marker, BALL_RED_MARKER_MAX, &red_n, "红");
    out->red_marker_count = (uint8_t)red_n;
}

void ball_detect_debug_ascii(char *out, int out_len)
{
    if (out == NULL || out_len <= 0) return;
    strncpy(out, s_debug_ascii, (size_t)out_len - 1);
    out[out_len - 1] = '\0';
}

void ball_detect_get_gray_diag(int *mn, int *mx, int *avg,
                               uint32_t *black_px, uint32_t *blue_px,
                               uint32_t *red_px)
{
    if (mn)  *mn  = s_diag_min;
    if (mx)  *mx  = s_diag_max;
    if (avg) *avg = s_diag_avg;
    if (black_px) *black_px = s_dbg_black_px;
    if (blue_px)  *blue_px  = s_dbg_blue_px;
    if (red_px)   *red_px   = s_dbg_red_px;
}
