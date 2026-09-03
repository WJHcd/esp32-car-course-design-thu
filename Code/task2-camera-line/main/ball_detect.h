#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 找球任务目标识别（红球 / 蓝球 / 绿色角标）
 *
 * 2026-09-03 场地实物变更：原目标"黑球"换成"红球"，红球完全复用
 * 原来黑球的识别链路（4× 降采样网格 + 连通域 + 形状过滤），黑色
 * 分类整类移除，红色分类改为按"球"处理；推球目标角标定为绿色
 * （红色/蓝色/黑色均被占用，绿色与场内元素全部正交），独立建类。
 *
 *   - 红球：场地以白底为主，红色与黑线/蓝线异色，纯色类本身干净；
 *     但球面高光、JPEG 色块会打碎色区，沿用 4× 网格 + 低格内门限
 *     保边缘，靠连通域把碎斑聚成整球，形状过滤剔除细长线段；
 *   - 蓝球与"禁止放球区"蓝色边界线同色，全局质心法会被线污染
 *     （质心落在两者之间）；连通域的形状指标（宽高比/填充率）能
 *     干净地区分"紧凑球团"与"细长线段"；
 *   - 绿角标：绿色在场内无任何同色源，纯色类干净；沿用原红色角标
 *     的宽松提取（不强制球形状，容纳远距离小块），最多输出两个。
 *
 * 输出（每帧）：
 *   red_ball  : 红球（最大合格连通域）
 *   blue_ball : 蓝球（最大合格连通域）
 *   marker[]  : 绿角标（最多两个，按面积从大到小，[0] 为较大者）
 *
 * 所有阈值集中在下方标定区，现场按光照调整。
 * ============================================================ */

/* ---- 颜色阈值标定区（2026-09-01 现场暗光标定；红球按新实物复标）---- */
#define BALL_GRID_DIV           4      /* 降采样倍数：每格 = 4x4 像素 */
#define BALL_CELL_HIT_MIN       4      /* 16 像素中至少 4 个命中；原来 8 个会吃掉小球边缘 */

/* 红球：R 通道判定。阈值沿用 camera-task 红球标定值（100/40，曾实球
 * 验证可用）。曾加过"G ≤ R 的 50%"比例门控来排棕色木板（增补 8），
 * 现已撤销：现场光照下红球 G 分量偏高被误拦；且棕色干扰的主要来源
 * （画面下方的车体/木板）已由底部 BALL_DETECT_Y_BOTTOM_PCT 屏蔽覆盖。 */
#define BALL_RED_R_MIN          100
#define BALL_RED_DELTA          40     /* R 需比 G 和 B 都高出此值 */
#define BALL_BLUE_MIN_B         50     /* 蓝：B 通道最小值（暗光下调低） */
#define BALL_BLUE_DELTA         20     /* 蓝：B 需比 max(R,G) 高出的差值 */

/* 绿角标：G 通道判定。起步值仿蓝球档（暗光保守），按实物与光照复标；
 * 选用饱和绿（草绿~深绿），勿用浅绿/黄绿（会贴近白/黄）。 */
#define BALL_GREEN_MIN_G        60
#define BALL_GREEN_DELTA        25     /* G 需比 max(R,B) 高出的差值 */

/* ---- 连通域形状过滤 ----
 * 小球在 240x160 解码帧里通常只有十几像素，不能先做强腐蚀。
 * 这里保留 4x4 网格以控制 RAM，用较低格内门限保留边缘，再靠
 * 连通域的紧凑度过滤细长赛道线。 */
#define BALL_ERODE_ENABLE       0      /* 诊断/当前实车：禁用会吞掉小球的强腐蚀 */
#define BALL_MIN_CELLS          3      /* 最少占用格子数（1 格 = 16 像素） */
#define BALL_MIN_BOX_CELLS      2      /* 宽、高至少各 2 格，排除单行/单列噪声 */
#define BALL_ASPECT_MAX         2.2f   /* max(宽,高)/min(宽,高)，球应接近 1 */
#define BALL_FILL_MIN           0.28f  /* 填充率下限（圆形小目标允许边缘缺口） */
#define BALL_MAX_BOX_WIDTH_PCT  55     /* 过大的色区/线段区域不作为球 */
#define BALL_MAX_BOX_HEIGHT_PCT 70

#define BALL_MARKER_MAX         2      /* 绿角标最多输出两个 */

/* ---- 检测区域 ----
 * 画面底部此百分比【不参与找球识别】：云台 35° 抬起时车头/车身自身
 * 占据画面下方约 20%，其颜色（棕/暗）会污染球分类（曾致误检）。
 * 注意与 ball_task 的 APPROACH 到位阈值联动：球可见区变为
 * y ∈ [0, 100-BALL_DETECT_Y_BOTTOM_PCT]，到位阈值须留出余量。 */
#define BALL_DETECT_Y_BOTTOM_PCT 20

/* 单个目标（球）的识别结果 */
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
    ball_blob_t red_ball;              /* 红球（原黑球角色：主目标） */
    ball_blob_t blue_ball;
    ball_blob_t marker[BALL_MARKER_MAX];   /* 绿角标，按面积从大到小 */
    uint8_t marker_count;
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
 * @brief 取最近一帧的调试 ASCII 视图（R=红球 L=蓝球 G=绿角标 .=空白）
 * @param out     输出缓冲（每 2 格采样 1 格，含换行）
 * @param out_len 缓冲长度
 */
void ball_detect_debug_ascii(char *out, int out_len);

/** @brief 取灰度诊断：中心区 min/max/avg + 全帧三类色像素计数 */
void ball_detect_get_gray_diag(int *mn, int *mx, int *avg,
                               uint32_t *red_px, uint32_t *blue_px,
                               uint32_t *green_px);
