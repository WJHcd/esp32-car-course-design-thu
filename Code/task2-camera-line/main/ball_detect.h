#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 找球任务目标识别（黑球 / 蓝球 / 红色角标）
 *
 * 改写自 camera-task 的颜色识别：在"逐像素分类"之上增加
 * 4× 降采样网格 + 连通域分析，原因：
 *   - 黑球与黑线同色，全局质心法会被线污染（质心落在两者之间）；
 *   - 蓝球与"禁止放球区"蓝色边界线同色，同理；
 *   - 连通域的形状指标（宽高比/填充率）能干净地区分
 *     "紧凑球团"与"细长线段"。
 *
 * 输出（每帧）：
 *   black_ball  : 黑球（最大合格连通域）
 *   blue_ball   : 蓝球（最大合格连通域）
 *   red_marker  : 红色角标（最多两个，按面积从大到小）
 *
 * 所有阈值集中在下方标定区，现场按光照调整。
 * ============================================================ */

/* ---- 颜色阈值标定区（2026-09-01 现场暗光标定）---- */
#define BALL_GRID_DIV           4      /* 降采样倍数：每格 = 4x4 像素 */
#define BALL_BLACK_GRAY_MAX     150    /* 黑：灰度低于此值判黑。2026-09-01
                                          * 100→150：实测球面呈深灰(~100-160)、
                                          * 原阈值只抓住最暗斑点凑不成团 */
#define BALL_BLUE_MIN_B         50     /* 蓝：B 通道最小值（暗光下调低） */
#define BALL_BLUE_DELTA         20     /* 蓝：B 需比 max(R,G) 高出的差值 */
#define BALL_RED_R_MIN          120    /* 红角标：R 通道最小值 */
#define BALL_RED_DELTA          40     /* 红角标：R 需比 G/B 高出的差值 */

/* ---- 连通域形状过滤（腐蚀后球团核心；阈值相应放宽）---- */
#define BALL_MIN_CELLS          3      /* 最少占用格子数（1 格 = 16 像素） */
#define BALL_ASPECT_MIN         0.35f  /* 包围盒宽高比下限（球 ≈ 1） */
#define BALL_ASPECT_MAX         3.5f   /* 包围盒宽高比上限 */
#define BALL_FILL_MIN           0.35f  /* 填充率下限（实际格数/包围盒面积） */

#define BALL_RED_MARKER_MAX     2      /* 红色角标最多输出两个 */

/* 单个目标（球或角标）的识别结果 */
typedef struct {
    bool detected;         /* 本帧是否找到合格目标 */
    float x_ratio;         /* 质心水平位置 [0,1]，0.5 = 画面正中 */
    float y_ratio;         /* 质心垂直位置 [0,1]，0 = 顶部 */
    float area_ratio;      /* 估算像素面积 / 全帧面积，越大越近 */
    uint16_t width_px;     /* 包围盒宽（像素，按降采样倍数还原） */
    uint16_t height_px;    /* 包围盒高 */
    uint16_t x_min_px;     /* 包围盒左上角 X（像素，供画框叠加） */
    uint16_t y_min_px;     /* 包围盒左上角 Y */
    uint32_t cell_count;   /* 占用格子数（诊断用） */
} ball_blob_t;

/* 一帧的找球总结果 */
typedef struct {
    ball_blob_t black_ball;
    ball_blob_t blue_ball;
    ball_blob_t red_marker[BALL_RED_MARKER_MAX];
    uint8_t red_marker_count;
} ball_field_result_t;

/**
 * @brief 初始化找球识别（记录图像尺寸，静态分配网格）
 */
void ball_detect_init(int width, int height);

/**
 * @brief 对一帧 RGB888 执行三色识别（覆盖写入 out）
 */
void ball_detect_process_rgb888(const uint8_t *rgb, ball_field_result_t *out);

/**
 * @brief 取最近一帧的调试 ASCII 视图（B=黑团 L=蓝团 R=红角标 .=空白）
 * @param out     输出缓冲（每 2 格采样 1 格，含换行）
 * @param out_len 缓冲长度
 */
void ball_detect_debug_ascii(char *out, int out_len);

/** @brief 取灰度诊断：中心区 min/max/avg + 全帧三色分类像素计数 */
void ball_detect_get_gray_diag(int *mn, int *mx, int *avg,
                               uint32_t *black_px, uint32_t *blue_px,
                               uint32_t *red_px);
