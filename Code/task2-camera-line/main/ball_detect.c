#include "ball_detect.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

/* ============================================================
 * 三色识别实现：逐像素分类 → 4× 降采样占用网格 → flood-fill
 * 连通域 → 形状过滤（宽高比/填充率）→ 每色取最大合格团块。
 * （2026-09-04：绿色终点改为独立 2x2 网格 + 空间/形状过滤 + 跨帧确认，
 *  同时兼顾远端小目标召回与画面下方车体误检抑制。）
 *
 * 每帧成本：全帧像素分类 ~0.5ms + 三张 80x60 网格的 flood fill
 * （合计 <4800 格）≈ 0.1ms，远小于 JPEG 解码。
 * ============================================================ */

#define GRID_W_MAX 80
#define GRID_H_MAX 60
#define MARKER_GRID_W_MAX 160
#define MARKER_GRID_H_MAX 120

static const char *TAG = "ball_detect";

static int s_width;
static int s_height;
static int s_grid_w;
static int s_grid_h;
static int s_marker_grid_w;
static int s_marker_grid_h;
static uint8_t *s_marker_occ;
static int16_t *s_marker_stack;
static size_t s_marker_capacity;

typedef struct {
    uint8_t occ[GRID_W_MAX * GRID_H_MAX];   /* 占用标记（1=该格该色命中） */
    int16_t stack[GRID_W_MAX * GRID_H_MAX]; /* flood fill 栈 */
} color_grid_t;

static color_grid_t s_grid_red;
static color_grid_t s_grid_blue;
static char s_debug_ascii[(GRID_W_MAX / 2 + 2) * (GRID_H_MAX / 2 + 1)];
static int s_diag_min, s_diag_max, s_diag_avg;   /* 中心区灰度诊断 */
static uint32_t s_dbg_red_px, s_dbg_blue_px, s_dbg_green_px;
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

    s_marker_grid_w = width / BALL_MARKER_GRID_DIV;
    s_marker_grid_h = height / BALL_MARKER_GRID_DIV;
    if (s_marker_grid_w > MARKER_GRID_W_MAX) s_marker_grid_w = MARKER_GRID_W_MAX;
    if (s_marker_grid_h > MARKER_GRID_H_MAX) s_marker_grid_h = MARKER_GRID_H_MAX;
    if (s_marker_grid_w < 1) s_marker_grid_w = 1;
    if (s_marker_grid_h < 1) s_marker_grid_h = 1;

    const size_t marker_cells = (size_t)s_marker_grid_w * s_marker_grid_h;
    if (marker_cells != s_marker_capacity) {
        uint8_t *new_occ = heap_caps_calloc(marker_cells, sizeof(uint8_t),
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        int16_t *new_stack = heap_caps_malloc(marker_cells * sizeof(int16_t),
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (new_occ == NULL || new_stack == NULL) {
            heap_caps_free(new_occ);
            heap_caps_free(new_stack);
            heap_caps_free(s_marker_occ);
            heap_caps_free(s_marker_stack);
            s_marker_occ = NULL;
            s_marker_stack = NULL;
            s_marker_capacity = 0;
            ESP_LOGE(TAG, "绿角标 2x2 网格申请失败（%u 格）",
                     (unsigned)marker_cells);
        } else {
            heap_caps_free(s_marker_occ);
            heap_caps_free(s_marker_stack);
            s_marker_occ = new_occ;
            s_marker_stack = new_stack;
            s_marker_capacity = marker_cells;
        }
    }
    memset(s_marker_tracks, 0, sizeof(s_marker_tracks));
    memset(&s_marker_diag, 0, sizeof(s_marker_diag));
}

/* ---- 像素级颜色分类（蓝/红/绿依次判定）---- */
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

static void extract_blobs(uint8_t *occ, int16_t *stack,
                          int grid_w, int grid_h, int grid_div,
                          ball_blob_t *out_blobs, int max_out, int *out_count,
                          bool require_ball_shape, int min_cells)
{
    *out_count = 0;
    if (occ == NULL || stack == NULL || grid_w <= 0 || grid_h <= 0) {
        return;
    }
    memset(stack, 0, (size_t)grid_w * grid_h * sizeof(*stack));

    for (int y0 = 0; y0 < grid_h; y0++) {
        for (int x0 = 0; x0 < grid_w; x0++) {
            const int idx = y0 * grid_w + x0;
            if (!occ[idx]) continue;

            /* flood fill：从该格吞掉整个连通域（吞掉的格子清零防重复） */
            blob_cells_t c = {0};
            int top = 0;
            stack[top++] = (int16_t)idx;
            occ[idx] = 0;
            c.min_x = c.max_x = x0;
            c.min_y = c.max_y = y0;
            while (top > 0) {
                const int cur = stack[--top];
                const int cx = cur % grid_w;
                const int cy = cur / grid_w;
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
                    if (nx < 0 || nx >= grid_w || ny < 0 || ny >= grid_h) continue;
                    const int nidx = ny * grid_w + nx;
                    if (occ[nidx]) {
                        occ[nidx] = 0;
                        stack[top++] = (int16_t)nidx;
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
                if (bw * grid_div * 100 > s_width * BALL_MAX_BOX_WIDTH_PCT) continue;
                if (bh * grid_div * 100 > s_height * BALL_MAX_BOX_HEIGHT_PCT) continue;
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
            out->x_ratio = ((float)c.sum_x / c.count + 0.5f) * grid_div / (float)s_width;
            out->y_ratio = ((float)c.sum_y / c.count + 0.5f) * grid_div / (float)s_height;
            out->width_px = (uint16_t)(bw * grid_div);
            out->height_px = (uint16_t)(bh * grid_div);
            out->x_min_px = (uint16_t)(c.min_x * grid_div);
            out->y_min_px = (uint16_t)(c.min_y * grid_div);
            out->area_ratio = (float)c.count * grid_div * grid_div /
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
    if (m->width_px * 100 > s_width * BALL_MARKER_MAX_BOX_WIDTH_PCT ||
        m->height_px * 100 > s_height * BALL_MARKER_MAX_BOX_HEIGHT_PCT) {
        return false;
    }

    /* 远端 1~3 格目标保留，交给三帧确认；较大的目标必须是二维色块，
     * 不能是车体高光/JPEG 产生的 1xN 细条。 */
    if (m->cell_count <= BALL_MARKER_SMALL_MAX_CELLS) {
        return true;
    }
    const int bw = m->width_px / BALL_MARKER_GRID_DIV;
    const int bh = m->height_px / BALL_MARKER_GRID_DIV;
    if (bw < 2 || bh < 2) {
        return false;
    }
    const float aspect = (float)(bw > bh ? bw : bh) /
                         (float)(bw < bh ? bw : bh);
    const float fill = (float)m->cell_count / (float)(bw * bh);
    return aspect <= BALL_MARKER_ASPECT_MAX && fill >= BALL_MARKER_FILL_MIN;
}

static bool marker_is_ball_fringe(const ball_blob_t *m, const ball_blob_t *ball)
{
    if (!ball->detected || m->cell_count > BALL_MARKER_FRINGE_MAX_CELLS) {
        return false;
    }

    /* 只删除中心真正落在球框内部的小绿块。旧版把球框向外膨胀 8px，
     * 三点共线时会连远处真角标一起删掉。 */
    const float mx = m->x_ratio * s_width;
    const float my = m->y_ratio * s_height;
    const int bx0 = ball->x_min_px;
    const int by0 = ball->y_min_px;
    const int bx1 = ball->x_min_px + ball->width_px;
    const int by1 = ball->y_min_px + ball->height_px;
    return mx >= bx0 && mx <= bx1 && my >= by0 && my <= by1;
}

static bool marker_can_start_track(const ball_blob_t *m,
                                   const ball_field_result_t *field)
{
    /* 车体固定在画面下部：这里仅限制“新目标建立”。已经从远处确认的
     * 真角标可随车辆接近继续跟踪到 0.70 检测边界。 */
    if (m->y_ratio > BALL_MARKER_NEW_TRACK_Y_MAX) {
        return false;
    }

    /* 正常推球几何中终点比球远，图像 y 应在球上方或至多略低。
     * 只要与任一可见球满足该关系即可，避免两个球高低不同时误杀。 */
    const ball_blob_t *balls[2] = { &field->red_ball, &field->blue_ball };
    bool any_ball = false;
    bool plausible_for_ball = false;
    for (int i = 0; i < 2; i++) {
        if (!balls[i]->detected) continue;
        any_ball = true;
        if (m->y_ratio <= balls[i]->y_ratio + BALL_MARKER_BELOW_BALL_TOL) {
            plausible_for_ball = true;
        }
    }
    return !any_ball || plausible_for_ball;
}

static uint8_t marker_required_hits(const ball_blob_t *m)
{
    return m->cell_count <= BALL_MARKER_SMALL_MAX_CELLS
        ? BALL_MARKER_SMALL_CONFIRM_FRAMES
        : BALL_MARKER_CONFIRM_FRAMES;
}

static void update_marker_tracks(const ball_blob_t *raw, int raw_n,
                                 ball_field_result_t *field)
{
    ball_blob_t candidates[BALL_MARKER_RAW_MAX];
    int candidate_n = 0;
    s_marker_diag.rejected_shape = 0;
    s_marker_diag.rejected_ball_fringe = 0;
    s_marker_diag.rejected_body_or_geometry = 0;

    for (int i = 0; i < raw_n; i++) {
        if (!marker_shape_ok(&raw[i])) {
            if (s_marker_diag.rejected_shape < UINT8_MAX) {
                s_marker_diag.rejected_shape++;
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
        candidates[candidate_n++] = raw[i];
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
            const float dx = fabsf(candidates[i].x_ratio - track->blob.x_ratio);
            const float dy = fabsf(candidates[i].y_ratio - track->blob.y_ratio);
            if (dx > BALL_MARKER_TRACK_MATCH_X ||
                dy > BALL_MARKER_TRACK_MATCH_Y) {
                continue;
            }
            const float cost = dx / BALL_MARKER_TRACK_MATCH_X +
                               dy / BALL_MARKER_TRACK_MATCH_Y;
            if (cost < best_cost) {
                best_cost = cost;
                best = i;
            }
        }

        if (best >= 0) {
            used[best] = true;
            track->blob = candidates[best];
            track->matched_this_frame = true;
            track->misses = 0;
            if (track->hits < UINT8_MAX) track->hits++;
            if (track->hits >= marker_required_hits(&track->blob)) {
                track->confirmed = true;
            }
        } else {
            if (track->misses < UINT8_MAX) track->misses++;
            if (!track->confirmed) {
                /* 首次确认要求连续帧；偶发噪声隔几帧重现不能累计过关。 */
                track->hits = 0;
            }
            if (track->misses > BALL_MARKER_TRACK_MAX_MISSES) {
                memset(track, 0, sizeof(*track));
            }
        }
    }

    /* 未匹配候选只允许在非车体区建立新轨迹；内部留 6 个轨迹槽，避免
     * 两个面积较大的噪声在确认前就挤掉真正的两个远端终点。 */
    for (int i = 0; i < candidate_n; i++) {
        if (used[i]) continue;
        if (!marker_can_start_track(&candidates[i], field)) {
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
        if (free_slot < 0) continue;
        marker_track_t *track = &s_marker_tracks[free_slot];
        memset(track, 0, sizeof(*track));
        track->active = true;
        track->matched_this_frame = true;
        track->hits = 1;
        track->blob = candidates[i];
        track->confirmed =
            track->hits >= marker_required_hits(&track->blob);
        used[i] = true;
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

        /* 稳定帧数优先、面积次之；持续稳定的远端小角标不会被单帧大
         * 反光挤出仅有的两个最终输出槽。 */
        const uint32_t score = (uint32_t)track->hits * 65536u +
                               track->blob.cell_count;
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

    /* 对外仍按面积排序，保持 ball_field_result_t 的既有约定。 */
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
    memset(&s_marker_diag, 0, sizeof(s_marker_diag));
    if (s_marker_occ != NULL && s_marker_capacity > 0) {
        memset(s_marker_occ, 0, s_marker_capacity * sizeof(*s_marker_occ));
    }

    /* 红/蓝球保持已经实车验证的 4x4 网格采样行为，避免本次绿色终点
     * 修改改变球识别。绿色在下方另走真正的 2x2 完整格累计。 */
    const int cells_x = s_grid_w, cells_y = s_grid_h;
    uint32_t tot_red = 0, tot_blue = 0, tot_green = 0;
    /* 原始画面底部 30% 不参与任何目标分类。 */
    const int y_max = s_height - s_height * BALL_DETECT_Y_BOTTOM_PCT / 100;

    for (int gy = 0; gy < cells_y; gy++) {
        for (int py = 0; py < BALL_GRID_DIV; py++) {
            const int y = gy * BALL_GRID_DIV + py;
            if (y >= s_height || y >= y_max) break;
            const uint8_t *row = rgb + (size_t)y * s_width * 3;
            for (int gx = 0; gx < cells_x; gx++) {
                int red = 0, blue = 0;
                for (int px = 0; px < BALL_GRID_DIV; px++) {
                    const int x = gx * BALL_GRID_DIV + px;
                    if (x >= s_width) break;
                    const uint8_t *p = row + (size_t)x * 3;
                    switch (pixel_class(p[0], p[1], p[2])) {
                    case 1: blue++; tot_blue++; break;
                    case 2: red++; tot_red++; break;
                    default: break;
                    }
                }
                if (red   >= BALL_CELL_HIT_MIN) s_grid_red.occ[gy * cells_x + gx] = 1;
                if (blue  >= BALL_CELL_HIT_MIN) s_grid_blue.occ[gy * cells_x + gx] = 1;
            }
        }
    }

    /* 绿色终点专用 2x2 网格。计数器包住 py+px 两层，确保门槛是真正
     * 的 N/4；旧实现把计数器放在 py 内，实际变成“任一横行 N/4”。 */
    if (s_marker_occ != NULL && s_marker_stack != NULL) {
        for (int gy = 0; gy < s_marker_grid_h; gy++) {
            for (int gx = 0; gx < s_marker_grid_w; gx++) {
                int green = 0;
                for (int py = 0; py < BALL_MARKER_GRID_DIV; py++) {
                    const int y = gy * BALL_MARKER_GRID_DIV + py;
                    if (y >= s_height || y >= y_max) break;
                    const uint8_t *row = rgb + (size_t)y * s_width * 3;
                    for (int px = 0; px < BALL_MARKER_GRID_DIV; px++) {
                        const int x = gx * BALL_MARKER_GRID_DIV + px;
                        if (x >= s_width) break;
                        const uint8_t *p = row + (size_t)x * 3;
                        if (pixel_class(p[0], p[1], p[2]) == 3) {
                            green++;
                            tot_green++;
                        }
                    }
                }
                if (green >= BALL_MARKER_CELL_HIT_MIN) {
                    s_marker_occ[gy * s_marker_grid_w + gx] = 1;
                    if (s_marker_diag.occupied_cells < UINT16_MAX) {
                        s_marker_diag.occupied_cells++;
                    }
                }
            }
        }
    }
    s_dbg_red_px = tot_red;
    s_dbg_blue_px = tot_blue;
    s_dbg_green_px = tot_green;
    s_marker_diag.raw_green_pixels = tot_green;

    /* 调试视图与头文件约定保持一致：R=红球、L=蓝球、G=绿角标、.=空。
     * 每字符覆盖 2x2 个球网格；绿色 2x2 网格按同一像素区域汇总。 */
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
                    }
                }
                if (s_marker_occ != NULL) {
                    const int px0 = gx * BALL_GRID_DIV;
                    int px1 = (gx + 2) * BALL_GRID_DIV;
                    const int py0 = gy * BALL_GRID_DIV;
                    int py1 = (gy + 2) * BALL_GRID_DIV;
                    if (px1 > s_width) px1 = s_width;
                    if (py1 > y_max) py1 = y_max;
                    const int mgx0 = px0 / BALL_MARKER_GRID_DIV;
                    const int mgy0 = py0 / BALL_MARKER_GRID_DIV;
                    const int mgx1 = (px1 + BALL_MARKER_GRID_DIV - 1) /
                                     BALL_MARKER_GRID_DIV;
                    const int mgy1 = (py1 + BALL_MARKER_GRID_DIV - 1) /
                                     BALL_MARKER_GRID_DIV;
                    for (int my = mgy0; my < mgy1 && my < s_marker_grid_h && !green; my++) {
                        for (int mx = mgx0; mx < mgx1 && mx < s_marker_grid_w; mx++) {
                            if (s_marker_occ[my * s_marker_grid_w + mx]) {
                                green = true;
                                break;
                            }
                        }
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
    if (BALL_ERODE_ENABLE) {
        erode_grid(&s_grid_red);
        erode_grid(&s_grid_blue);
    }
    extract_blobs(s_grid_red.occ, s_grid_red.stack,
                  s_grid_w, s_grid_h, BALL_GRID_DIV,
                  &out->red_ball, 1, &n, true, BALL_MIN_CELLS);
    extract_blobs(s_grid_blue.occ, s_grid_blue.stack,
                  s_grid_w, s_grid_h, BALL_GRID_DIV,
                  &out->blue_ball, 1, &n, true, BALL_MIN_CELLS);

    /* 先取最多 8 个绿色原始连通域，再经形状、球邻域、车体区域和
     * 跨帧轨迹筛成最终两个，避免大假块提前挤掉远端小目标。 */
    ball_blob_t raw_marker[BALL_MARKER_RAW_MAX] = {0};
    int marker_n = 0;
    if (s_marker_occ != NULL && s_marker_stack != NULL) {
        extract_blobs(s_marker_occ, s_marker_stack,
                      s_marker_grid_w, s_marker_grid_h, BALL_MARKER_GRID_DIV,
                      raw_marker, BALL_MARKER_RAW_MAX, &marker_n,
                      false, BALL_MARKER_MIN_CELLS);
    }
    s_marker_diag.raw_candidates = (uint8_t)marker_n;
    update_marker_tracks(raw_marker, marker_n, out);
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

void ball_detect_get_marker_diag(ball_marker_diag_t *out)
{
    if (out != NULL) {
        *out = s_marker_diag;
    }
}
