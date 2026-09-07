#include "ball_detect.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * 三色识别实现：逐像素分类 → 4× 降采样占用网格 → flood-fill
 * 连通域 → 形状过滤（宽高比/填充率）→ 每色取最大合格团块。
 * （黑终点使用灰度分割 + 实心方块/白地包围/球相对位置过滤
 *  + 候选位置跨帧确认；
 *  红/蓝球沿用既有绝对差判据与实车标定。）
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
static uint32_t s_dbg_red_px, s_dbg_blue_px, s_dbg_marker_px;
static ball_marker_diag_t s_marker_diag;

typedef struct {
    bool active;
    bool confirmed;
    bool matched_this_frame;
    uint8_t hits;
    uint8_t misses;
    ball_blob_t blob;
} marker_track_t;

static marker_track_t s_marker_tracks[BALL_MARKER_TRACK_MAX];

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
    memset(s_marker_tracks, 0, sizeof(s_marker_tracks));
    memset(&s_marker_diag, 0, sizeof(s_marker_diag));
}

/* ---- 像素级球颜色分类（黑终点另走灰度判定）---- */
static inline int pixel_class(uint8_t r, uint8_t g, uint8_t b)
{
    if (b > BALL_BLUE_MIN_B && b - (r > g ? r : g) > BALL_BLUE_DELTA) {
        return 1;                                   /* 蓝 */
    }
    if (r > BALL_RED_R_MIN && r - g > BALL_RED_DELTA && r - b > BALL_RED_DELTA) {
        return 2;                                   /* 红 */
    }
    return 0;
}

static inline uint8_t pixel_gray(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint8_t)(((uint16_t)r * 77u + (uint16_t)g * 150u +
                      (uint16_t)b * 29u) >> 8);
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
 * min_cells 由调用方传入（球与黑终点分别配置） */
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

                /* 8 邻域比 4 邻域更适合圆球和远端稀疏色块：斜向相接的
                 * 有效像素不应被拆成多个不足面积的小连通域。 */
                for (int dy = -1; dy <= 1; dy++) {
                    for (int dx = -1; dx <= 1; dx++) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = cx + dx;
                        const int ny = cy + dy;
                        if (nx < 0 || nx >= s_grid_w ||
                            ny < 0 || ny >= s_grid_h) continue;
                        const int nidx = ny * s_grid_w + nx;
                        if (g->occ[nidx]) {
                            g->occ[nidx] = 0;
                            g->stack[top++] = nidx;
                        }
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

static bool marker_shape_ok(const ball_blob_t *m)
{
    const int edge_x = s_width * BALL_MARKER_EDGE_MARGIN_PCT / 100;
    const int edge_y = s_height * BALL_MARKER_EDGE_MARGIN_PCT / 100;
    if (m->x_min_px <= edge_x || m->y_min_px <= edge_y ||
        m->x_min_px + m->width_px >= s_width - edge_x ||
        m->y_min_px + m->height_px >= s_height - edge_y) {
        return false; /* 场外家具/黑线从画面边缘延入时不构成完整终点 */
    }
    if (m->width_px * 100 > s_width * BALL_MARKER_MAX_BOX_WIDTH_PCT ||
        m->height_px * 100 > s_height * BALL_MARKER_MAX_BOX_HEIGHT_PCT) {
        return false;
    }
    const int bw = m->width_px / BALL_GRID_DIV;
    const int bh = m->height_px / BALL_GRID_DIV;
    if (bw < 2 || bh < 2) return false;
    const float aspect = (float)(bw > bh ? bw : bh) /
                         (float)(bw < bh ? bw : bh);
    const float fill = (float)m->cell_count / (float)(bw * bh);
    return aspect <= BALL_MARKER_ASPECT_MAX && fill >= BALL_MARKER_FILL_MIN;
}

/* 黑终点应落在浅色赛场地面上。只采样包围盒外环，不要求四边全白，
 * 因为球接近终点时会遮住一部分；细线主要仍由形状过滤负责。 */
static bool marker_has_light_surround(const uint8_t *rgb,
                                      const ball_blob_t *m)
{
    const int margin = BALL_MARKER_SURROUND_MARGIN_CELLS * BALL_GRID_DIV;
    int x0 = (int)m->x_min_px - margin;
    int y0 = (int)m->y_min_px - margin;
    int x1 = (int)m->x_min_px + m->width_px + margin - 1;
    int y1 = (int)m->y_min_px + m->height_px + margin - 1;
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= s_width) x1 = s_width - 1;
    if (y1 >= s_height) y1 = s_height - 1;

    int samples = 0, light = 0;
    for (int y = y0; y <= y1; y += 2) {
        for (int x = x0; x <= x1; x += 2) {
            const bool outside = x < m->x_min_px ||
                x >= m->x_min_px + m->width_px ||
                y < m->y_min_px || y >= m->y_min_px + m->height_px;
            if (!outside) continue;
            const uint8_t *p = rgb + ((size_t)y * s_width + x) * 3;
            samples++;
            if (pixel_gray(p[0], p[1], p[2]) >=
                BALL_MARKER_SURROUND_GRAY_MIN) light++;
        }
    }
    return samples > 0 &&
           light * 100 >= samples * BALL_MARKER_SURROUND_LIGHT_PCT;
}

static bool marker_is_ball_fringe(const ball_blob_t *m,
                                  const ball_blob_t *ball)
{
    if (!ball->detected || m->cell_count > BALL_MARKER_FRINGE_MAX_CELLS) {
        return false;
    }
    const float mx = m->x_ratio * s_width;
    const float my = m->y_ratio * s_height;
    return mx >= ball->x_min_px &&
           mx <= ball->x_min_px + ball->width_px &&
           my >= ball->y_min_px &&
           my <= ball->y_min_px + ball->height_px;
}

static bool marker_plausible_with_ball(const ball_blob_t *m,
                                       const ball_field_result_t *field)
{
    /* 黑色干扰太多，终点不再允许脱离球单独成立。它必须位于至少一个
     * 可见球的后方（图像中更上方，近距离遮挡留少量容差）。 */
    const ball_blob_t *visible[2] = { &field->red_ball, &field->blue_ball };
    bool any_ball = false;
    bool plausible = false;
    for (int i = 0; i < 2; i++) {
        if (!visible[i]->detected) continue;
        any_ball = true;
        if (m->y_ratio <= visible[i]->y_ratio + BALL_MARKER_BELOW_BALL_TOL) {
            plausible = true;
        }
    }
    return any_ball && plausible;
}

static bool marker_can_start_track(const ball_blob_t *m,
                                   const ball_field_result_t *field)
{
    /* 场外桌椅通常落在极上方或触边；车体在下方。已确认的真实终点
     * 可继续跟踪到统一的 y=0.70 分类边界。 */
    if (m->y_ratio < BALL_MARKER_NEW_TRACK_Y_MIN ||
        m->y_ratio > BALL_MARKER_NEW_TRACK_Y_MAX) return false;
    return marker_plausible_with_ball(m, field);
}

static uint8_t marker_required_hits(const ball_blob_t *m)
{
    return m->cell_count <= BALL_MARKER_SMALL_MAX_CELLS
        ? BALL_MARKER_SMALL_CONFIRM_FRAMES
        : BALL_MARKER_CONFIRM_FRAMES;
}

static void update_marker_tracks(const uint8_t *rgb,
                                 const ball_blob_t *raw, int raw_n,
                                 ball_field_result_t *field)
{
    ball_blob_t candidate[BALL_MARKER_RAW_MAX];
    int candidate_n = 0;
    s_marker_diag.rejected_shape = 0;
    s_marker_diag.rejected_surround = 0;
    s_marker_diag.rejected_ball_fringe = 0;
    s_marker_diag.rejected_body_or_geometry = 0;

    for (int i = 0; i < raw_n; i++) {
        if (!marker_shape_ok(&raw[i])) {
            if (s_marker_diag.rejected_shape < UINT8_MAX) {
                s_marker_diag.rejected_shape++;
            }
            continue;
        }
        if (!marker_has_light_surround(rgb, &raw[i])) {
            if (s_marker_diag.rejected_surround < UINT8_MAX) {
                s_marker_diag.rejected_surround++;
            }
            continue;
        }
        if (marker_is_ball_fringe(&raw[i], &field->red_ball) ||
            marker_is_ball_fringe(&raw[i], &field->blue_ball)) {
            if (s_marker_diag.rejected_ball_fringe < UINT8_MAX) {
                s_marker_diag.rejected_ball_fringe++;
            }
            continue;
        }
        if (!marker_plausible_with_ball(&raw[i], field)) {
            if (s_marker_diag.rejected_body_or_geometry < UINT8_MAX) {
                s_marker_diag.rejected_body_or_geometry++;
            }
            continue;
        }
        candidate[candidate_n++] = raw[i];
    }

    bool used[BALL_MARKER_RAW_MAX] = {0};
    for (int t = 0; t < BALL_MARKER_TRACK_MAX; t++) {
        marker_track_t *track = &s_marker_tracks[t];
        track->matched_this_frame = false;
        if (!track->active) continue;

        int best = -1;
        float best_cost = 1e9f;
        for (int i = 0; i < candidate_n; i++) {
            if (used[i]) continue;
            const float dx = fabsf(candidate[i].x_ratio - track->blob.x_ratio);
            const float dy = fabsf(candidate[i].y_ratio - track->blob.y_ratio);
            if (dx > BALL_MARKER_TRACK_MATCH_X ||
                dy > BALL_MARKER_TRACK_MATCH_Y) continue;
            const float cost = dx / BALL_MARKER_TRACK_MATCH_X +
                               dy / BALL_MARKER_TRACK_MATCH_Y;
            if (cost < best_cost) {
                best_cost = cost;
                best = i;
            }
        }

        if (best >= 0) {
            used[best] = true;
            track->blob = candidate[best];
            track->matched_this_frame = true;
            track->misses = 0;
            if (track->hits < UINT8_MAX) track->hits++;
            if (track->hits >= marker_required_hits(&track->blob)) {
                track->confirmed = true;
            }
        } else {
            if (track->misses < UINT8_MAX) track->misses++;
            track->hits = 0;
            track->confirmed = false; /* 中断后重新连续确认，禁止继承旧资格 */
            if (track->misses > BALL_MARKER_TRACK_MAX_MISSES) {
                memset(track, 0, sizeof(*track));
            }
        }
    }

    for (int i = 0; i < candidate_n; i++) {
        if (used[i]) continue;
        if (!marker_can_start_track(&candidate[i], field)) {
            if (s_marker_diag.rejected_body_or_geometry < UINT8_MAX) {
                s_marker_diag.rejected_body_or_geometry++;
            }
            continue;
        }
        int free_slot = -1;
        for (int t = 0; t < BALL_MARKER_TRACK_MAX; t++) {
            if (!s_marker_tracks[t].active) {
                free_slot = t;
                break;
            }
        }
        if (free_slot < 0) {
            /* 旧位置短暂保留用于关联，但不能占满轨迹池挡住新终点。 */
            for (int t = 0; t < BALL_MARKER_TRACK_MAX; t++) {
                if (!s_marker_tracks[t].matched_this_frame &&
                    (free_slot < 0 || s_marker_tracks[t].misses >
                     s_marker_tracks[free_slot].misses)) free_slot = t;
            }
        }
        if (free_slot < 0) continue;
        marker_track_t *track = &s_marker_tracks[free_slot];
        memset(track, 0, sizeof(*track));
        track->active = true;
        track->matched_this_frame = true;
        track->hits = 1;
        track->blob = candidate[i];
        track->confirmed = track->hits >= marker_required_hits(&track->blob);
    }

    field->marker_count = 0;
    uint32_t out_score[BALL_MARKER_MAX] = {0};
    s_marker_diag.pending_tracks = 0;
    for (int t = 0; t < BALL_MARKER_TRACK_MAX; t++) {
        const marker_track_t *track = &s_marker_tracks[t];
        if (!track->active || !track->matched_this_frame) continue;
        if (!track->confirmed) {
            if (s_marker_diag.pending_tracks < UINT8_MAX) {
                s_marker_diag.pending_tracks++;
            }
            continue;
        }
        /* 连续确认只是准入条件；寿命不参与排序，避免旧假目标压住真目标。 */
        const uint32_t score = track->blob.cell_count;
        int slot = -1;
        if (field->marker_count < BALL_MARKER_MAX) {
            slot = field->marker_count++;
        } else {
            int weakest = 0;
            for (int i = 1; i < BALL_MARKER_MAX; i++) {
                if (out_score[i] < out_score[weakest]) weakest = i;
            }
            if (score > out_score[weakest]) slot = weakest;
        }
        if (slot >= 0) {
            field->marker[slot] = track->blob;
            out_score[slot] = score;
        }
    }
    if (field->marker_count == 2 &&
        field->marker[1].cell_count > field->marker[0].cell_count) {
        const ball_blob_t tmp = field->marker[0];
        field->marker[0] = field->marker[1];
        field->marker[1] = tmp;
    }
    s_marker_diag.confirmed_markers = field->marker_count;
}

void ball_detect_process_rgb888(const uint8_t *rgb, ball_field_result_t *out)
{
    memset(&s_grid_red, 0, sizeof(s_grid_red));
    memset(&s_grid_blue, 0, sizeof(s_grid_blue));
    memset(&s_grid_marker, 0, sizeof(s_grid_marker));
    memset(&s_marker_diag, 0, sizeof(s_marker_diag));

    /* 逐像素分类并落到降采样网格。球的圆弧边缘、高光和 JPEG 色块
     * 会让格内颜色不纯，使用较低命中数保住小目标，噪声交给连通域
     * 的面积/紧凑度过滤。黑终点独立按灰度生成候选，再跨帧确认。 */
    const int cells_x = s_grid_w, cells_y = s_grid_h;
    uint32_t tot_red = 0, tot_blue = 0, tot_marker = 0;
    /* 底部车体区域不参与分类（画面下方约 30% 是车自身，见 ball_detect.h） */
    const int y_max = s_height - s_height * BALL_DETECT_Y_BOTTOM_PCT / 100;
    for (int gy = 0; gy < cells_y; gy++) {
        for (int gx = 0; gx < cells_x; gx++) {
            int red = 0, blue = 0, marker_black = 0;
            for (int py = 0; py < BALL_GRID_DIV; py++) {
                const int y = gy * BALL_GRID_DIV + py;
                if (y >= s_height || y >= y_max) break;
                const uint8_t *row = rgb + (size_t)y * s_width * 3;
                for (int px = 0; px < BALL_GRID_DIV; px++) {
                    const int x = gx * BALL_GRID_DIV + px;
                    if (x >= s_width) break;
                    const uint8_t *p = row + (size_t)x * 3;
                    const int cls = pixel_class(p[0], p[1], p[2]);
                    switch (cls) {
                    case 1: blue++; tot_blue++; break;
                    case 2: red++; tot_red++; break;
                    default: break;
                    }
                    /* 红/蓝球在灰度上也可能很暗；已被颜色类命中的像素
                     * 不得同时进入黑终点掩码。 */
                    if (cls == 0 && pixel_gray(p[0], p[1], p[2]) <=
                        BALL_MARKER_BLACK_GRAY_MAX) {
                        marker_black++;
                        tot_marker++;
                    }
                }
            }
            const int idx = gy * cells_x + gx;
            if (red  >= BALL_CELL_HIT_MIN) s_grid_red.occ[idx] = 1;
            if (blue >= BALL_CELL_HIT_MIN) s_grid_blue.occ[idx] = 1;
            /* 黑终点格内要求至少半数像素为黑；时序确认在候选轨迹层。 */
            if (marker_black >= BALL_MARKER_CELL_HIT_MIN) {
                s_grid_marker.occ[idx] = 1;
                if (s_marker_diag.occupied_cells < UINT16_MAX) {
                    s_marker_diag.occupied_cells++;
                }
            }
        }
    }
    s_dbg_red_px = tot_red;
    s_dbg_blue_px = tot_blue;
    s_dbg_marker_px = tot_marker;
    s_marker_diag.raw_marker_pixels = tot_marker;

    /* 调试视图与头文件约定保持一致：R=红球、L=蓝球、M=黑终点、.=空。
     * 每个字符汇总 2x2 个网格，方便直接判断颜色分类是否产生候选。 */
    {
        char *p = s_debug_ascii;
        for (int gy = 0; gy < cells_y; gy += 2) {
            for (int gx = 0; gx < cells_x; gx += 2) {
                bool red = false, blue = false, marker = false;
                for (int dy = 0; dy < 2 && gy + dy < cells_y; dy++) {
                    for (int dx = 0; dx < 2 && gx + dx < cells_x; dx++) {
                        const int idx = (gy + dy) * cells_x + gx + dx;
                        red   |= s_grid_red.occ[idx] != 0;
                        blue  |= s_grid_blue.occ[idx] != 0;
                        marker |= s_grid_marker.occ[idx] != 0;
                    }
                }
                *p++ = red ? 'R' : (blue ? 'L' : (marker ? 'M' : '.'));
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
    /* 先保留更多原始黑色候选；空间、形状和连续帧确认随后统一处理。 */
    ball_blob_t raw_marker[BALL_MARKER_RAW_MAX] = {0};
    int marker_n = 0;
    extract_blobs(&s_grid_marker, raw_marker, BALL_MARKER_RAW_MAX, &marker_n,
                  false, BALL_MARKER_MIN_CELLS);
    s_marker_diag.raw_candidates = (uint8_t)marker_n;
    update_marker_tracks(rgb, raw_marker, marker_n, out);
}

void ball_detect_debug_ascii(char *out, int out_len)
{
    if (out == NULL || out_len <= 0) return;
    strncpy(out, s_debug_ascii, (size_t)out_len - 1);
    out[out_len - 1] = '\0';
}

void ball_detect_get_gray_diag(int *mn, int *mx, int *avg,
                               uint32_t *red_px, uint32_t *blue_px,
                               uint32_t *marker_px)
{
    if (mn)  *mn  = s_diag_min;
    if (mx)  *mx  = s_diag_max;
    if (avg) *avg = s_diag_avg;
    if (red_px)   *red_px   = s_dbg_red_px;
    if (blue_px)  *blue_px  = s_dbg_blue_px;
    if (marker_px) *marker_px = s_dbg_marker_px;
}

void ball_detect_get_marker_diag(ball_marker_diag_t *out)
{
    if (out != NULL) {
        *out = s_marker_diag;
    }
}
