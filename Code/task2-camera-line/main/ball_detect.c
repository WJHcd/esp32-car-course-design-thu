#include "ball_detect.h"
#include <string.h>

/* ============================================================
 * 三色识别实现：逐像素分类 → 4× 降采样占用网格 → flood-fill
 * 连通域 → 形状过滤（宽高比/填充率）→ 每色取最大合格团块。
 * （2026-09-03：原黑球改红球，红色类改按球提取；角标改为黑色纸板，
 *  绿色独立建类提取（远端放宽），红球旁的 JPEG 绿边伪源由"红球邻域剔除"兜底。）
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

static color_grid_t s_grid_red;
static color_grid_t s_grid_blue;
static color_grid_t s_grid_marker;
static char s_debug_ascii[(GRID_W_MAX / 2 + 2) * (GRID_H_MAX / 2 + 1)];
static int s_diag_min, s_diag_max, s_diag_avg;   /* 中心区灰度诊断 */
static uint32_t s_dbg_red_px, s_dbg_blue_px, s_dbg_green_px;

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

/* ---- 像素级颜色分类（蓝/红/黑依次判定；黑放最后防吃掉彩色暗部）---- */
static inline int pixel_class(uint8_t r, uint8_t g, uint8_t b)
{
    if (b > BALL_BLUE_MIN_B && b - (r > g ? r : g) > BALL_BLUE_DELTA) {
        return 1;                                   /* 蓝 */
    }
    if (r > BALL_RED_R_MIN && r - g > BALL_RED_DELTA && r - b > BALL_RED_DELTA) {
        return 2;                                   /* 红 */
    }
    if (g > BALL_GREEN_MIN_G && g - (r > b ? r : b) > BALL_GREEN_DELTA) {
        return 3;                                   /* 绿（角标） */
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

/* ---- 单色连通域提取：从占用网格里 flood fill 出所有合格团块 ----
 * min_cells 由调用方传入（球 3 格；远端绿角标放宽到 2 格） */
typedef struct {
    int count;          /* 格子数 */
    int sum_x, sum_y;   /* 格子坐标和（求质心） */
    int min_x, min_y, max_x, max_y;
} blob_cells_t;

static void extract_blobs(color_grid_t *g, ball_blob_t *out_blobs,
                          int max_out, int *out_count, bool require_ball_shape,
                          int min_cells)
{
    *out_count = 0;
    memset(g->stack, 0, sizeof(g->stack));
    if (BALL_ERODE_ENABLE) {
        erode_grid(g);
    }

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

            /* 形状过滤：紧凑球团 vs 细长赛道线/边界线。aspect 的定义
             * 恒 >=1，因此只需要上限；原来的 0.35 下限永远不会生效。 */
            if (c.count < min_cells) continue;
            const int bw = c.max_x - c.min_x + 1;
            const int bh = c.max_y - c.min_y + 1;
            const float aspect = (float)(bw > bh ? bw : bh) / (bw < bh ? bw : bh);
            const float fill = (float)c.count / (float)(bw * bh);
            if (require_ball_shape) {
                if (bw < BALL_MIN_BOX_CELLS || bh < BALL_MIN_BOX_CELLS) continue;
                if (aspect > BALL_ASPECT_MAX) continue;
                if (fill < BALL_FILL_MIN) continue;
                if (bw * BALL_GRID_DIV * 100 > s_width * BALL_MAX_BOX_WIDTH_PCT) continue;
                if (bh * BALL_GRID_DIV * 100 > s_height * BALL_MAX_BOX_HEIGHT_PCT) continue;
            }

            int slot;
            if (*out_count < max_out) {
                slot = (*out_count)++;
            } else {
                /* 已满：替换最小的那个（保持最大的前 max_out 个） */
                int smallest = 0;
                for (int i = 1; i < max_out; i++) {
                    if (out_blobs[i].cell_count < out_blobs[smallest].cell_count) {
                        smallest = i;
                    }
                }
                if (c.count <= (int)out_blobs[smallest].cell_count) continue;
                slot = smallest;
            }

            ball_blob_t *out = &out_blobs[slot];
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

        }
    }

    /* 按面积从大到小排序（保证同色候选多时 [0] 恒为最大者） */
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
    memset(&s_grid_red, 0, sizeof(s_grid_red));
    memset(&s_grid_blue, 0, sizeof(s_grid_blue));
    memset(&s_grid_marker, 0, sizeof(s_grid_marker));

    /* 逐像素分类并落到降采样网格。球的圆弧边缘、高光和 JPEG 色块
     * 会让格内颜色不纯，使用较低命中数保住小目标，噪声交给连通域
     * 的面积/紧凑度过滤。 */
    const int cells_x = s_grid_w, cells_y = s_grid_h;
    uint32_t tot_red = 0, tot_blue = 0, tot_green = 0;
    /* 底部车体区域不参与分类（画面下方约 20% 是车自身，见 ball_detect.h） */
    const int y_max = s_height - s_height * BALL_DETECT_Y_BOTTOM_PCT / 100;

    for (int gy = 0; gy < cells_y; gy++) {
        for (int py = 0; py < BALL_GRID_DIV; py++) {
            const int y = gy * BALL_GRID_DIV + py;
            if (y >= s_height || y >= y_max) break;
            const uint8_t *row = rgb + (size_t)y * s_width * 3;
            for (int gx = 0; gx < cells_x; gx++) {
                int red = 0, blue = 0, green = 0;
                for (int px = 0; px < BALL_GRID_DIV; px++) {
                    const int x = gx * BALL_GRID_DIV + px;
                    if (x >= s_width) break;
                    const uint8_t *p = row + (size_t)x * 3;
                    switch (pixel_class(p[0], p[1], p[2])) {
                    case 1: blue++; tot_blue++; break;
                    case 2: red++; tot_red++; break;
                    case 3: green++; tot_green++; break;
                    default: break;
                    }
                }
                if (red   >= BALL_CELL_HIT_MIN) s_grid_red.occ[gy * cells_x + gx] = 1;
                if (blue  >= BALL_CELL_HIT_MIN) s_grid_blue.occ[gy * cells_x + gx] = 1;
                /* 绿角标在远端成像极小：格内命中数单独放宽（3/16） */
                if (green >= BALL_MARKER_CELL_HIT_MIN)
                    s_grid_marker.occ[gy * cells_x + gx] = 1;
            }
        }
    }
    s_dbg_red_px = tot_red;
    s_dbg_blue_px = tot_blue;
    s_dbg_green_px = tot_green;

    /* 调试视图与头文件约定保持一致：R=红球、L=蓝球、G=绿角标、.=空。
     * 每个字符汇总 2x2 个网格，方便直接判断颜色分类是否产生候选。 */
    {
        char *p = s_debug_ascii;
        for (int gy = 0; gy < cells_y; gy += 2) {
            for (int gx = 0; gx < cells_x; gx += 2) {
                bool red = false, blue = false, green = false;
                for (int dy = 0; dy < 2 && gy + dy < cells_y; dy++) {
                    for (int dx = 0; dx < 2 && gx + dx < cells_x; dx++) {
                        const int idx = (gy + dy) * cells_x + gx + dx;
                        red   |= s_grid_red.occ[idx] != 0;
                        blue  |= s_grid_blue.occ[idx] != 0;
                        green |= s_grid_marker.occ[idx] != 0;
                    }
                }
                *p++ = red ? 'R' : (blue ? 'L' : (green ? 'G' : '.'));
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
    extract_blobs(&s_grid_red, &out->red_ball, 1, &n, true, BALL_MIN_CELLS);
    extract_blobs(&s_grid_blue, &out->blue_ball, 1, &n, true, BALL_MIN_CELLS);
    /* 绿角标：绿色类是场上唯一绿色源，用宽松形状提取（无宽高比/填充率
     * 门槛，容纳远端 1~3 格的小成像），最少格数单独放宽到 2，取前二。 */
    int marker_n = 0;
    extract_blobs(&s_grid_marker, out->marker, BALL_MARKER_MAX, &marker_n,
                  false, BALL_MARKER_MIN_CELLS);
    out->marker_count = (uint8_t)marker_n;

    /* 球邻域剔除（红+蓝）：白底→彩色球的高对比边缘上，JPEG 压缩会
     * 打出绿色镶边伪源、贴着球形成小块（蓝球被推到车头附近时尤其
     * 明显——伪源出现在画面下部，曾被当成角标）。真角标远离球；
     * 球推到角标旁时角标成像也远大于镶边块——因此只剔除"与球包围盒
     * （膨胀 2 格）相交且较小（≤BALL_MARKER_FRINGE_MAX_CELLS 格）"
     * 的候选，大块真角标保留，完成判定不受影响。 */
    if (out->marker_count > 0) {
        int bx0[2] = {0, 0}, bx1[2] = {0, 0}, by0[2] = {0, 0}, by1[2] = {0, 0};
        int n_boxes = 0;
        const ball_blob_t *ball_ref[2] = { &out->red_ball, &out->blue_ball };
        for (int i = 0; i < 2; i++) {
            if (!ball_ref[i]->detected) continue;
            bx0[n_boxes] = (int)ball_ref[i]->x_min_px / BALL_GRID_DIV - 2;
            bx1[n_boxes] = (int)(ball_ref[i]->x_min_px + ball_ref[i]->width_px)
                           / BALL_GRID_DIV + 2;
            by0[n_boxes] = (int)ball_ref[i]->y_min_px / BALL_GRID_DIV - 2;
            by1[n_boxes] = (int)(ball_ref[i]->y_min_px + ball_ref[i]->height_px)
                           / BALL_GRID_DIV + 2;
            n_boxes++;
        }
        if (n_boxes > 0) {
            uint8_t kept = 0;
            for (uint8_t i = 0; i < out->marker_count; i++) {
                const ball_blob_t *m = &out->marker[i];
                const int mx0 = (int)m->x_min_px / BALL_GRID_DIV;
                const int mx1 = (int)(m->x_min_px + m->width_px) / BALL_GRID_DIV;
                const int my0 = (int)m->y_min_px / BALL_GRID_DIV;
                const int my1 = (int)(m->y_min_px + m->height_px) / BALL_GRID_DIV;
                bool hits_ball = false;
                for (int b = 0; b < n_boxes && !hits_ball; b++) {
                    hits_ball = (mx0 <= bx1[b] && mx1 >= bx0[b] &&
                                 my0 <= by1[b] && my1 >= by0[b]);
                }
                if (!hits_ball || m->cell_count > BALL_MARKER_FRINGE_MAX_CELLS) {
                    out->marker[kept++] = *m;
                }
            }
            out->marker_count = kept;
        }
    }
}

void ball_detect_debug_ascii(char *out, int out_len)
{
    if (out == NULL || out_len <= 0) return;
    strncpy(out, s_debug_ascii, (size_t)out_len - 1);
    out[out_len - 1] = '\0';
}

void ball_detect_get_gray_diag(int *mn, int *mx, int *avg,
                               uint32_t *red_px, uint32_t *blue_px,
                               uint32_t *green_px)
{
    if (mn)  *mn  = s_diag_min;
    if (mx)  *mx  = s_diag_max;
    if (avg) *avg = s_diag_avg;
    if (red_px)   *red_px   = s_dbg_red_px;
    if (blue_px)  *blue_px  = s_dbg_blue_px;
    if (green_px) *green_px = s_dbg_green_px;
}
