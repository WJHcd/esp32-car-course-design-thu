#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 找球任务目标识别（红球 / 蓝球 / 绿色角标）
 *
 * 2026-09-03 场地实物变更：原目标"黑球"换成"红球"，红球完全复用
 * 原来黑球的识别链路（4× 降采样网格 + 连通域 + 形状过滤），红色
 * 分类改为按"球"处理。推球目标角标为绿色（曾短暂改黑后又换回）。
 *
 *   - 红球：场地以白底为主，红色与黑线/蓝线异色，纯色类本身干净；
 *     但球面高光、JPEG 色块会打碎色区，沿用 4× 网格 + 低格内门限
 *     保边缘，靠连通域把碎斑聚成整球，形状过滤剔除细长线段；
 *   - 蓝球与"禁止放球区"蓝色边界线同色，全局质心法会被线污染
 *     （质心落在两者之间）；连通域的形状指标（宽高比/填充率）能
 *     干净地区分"紧凑球团"与"细长线段"；
 *   - 绿角标：归一化过量绿（nExG）比例判据，对亮度和白底稀释不变，
 *     解决"远端淡绿检不出、放宽又误检"的两难；只在上半区（角标在
 *     赛道最远处）按时序累积（IIR）产生候选，瞬态噪声冲不过门限；
 *     角标旁的 JPEG 绿边伪源由"球邻域剔除"兜底（见 ball_detect.c）。
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

/* 绿角标：归一化过量绿（nExG）比例判定（2026-09-04 调研后重构）。
 * 旧判据"G>阈值 且 G−max(R,B)>差值"是绝对量：角标在远端只占几像素，
 * 每个像素都是绿纸+白底的混合，绝对差随覆盖率线性衰减（实拍均值
 * RGB≈(134,158,102)：纯纸 G−max=24，25% 覆盖率时只剩 5）——收紧就漏
 * 远端淡绿、放宽又吃进暗区伪绿，两难无解。
 * 比例判据 nExG=(2G−R−B)/(R+G+B) 对亮度和白底稀释天然不变：
 * 实拍纯纸 0.20~0.25、25% 稀释仍有 0.03；白墙/地板/灰影 ≤0.01。
 * 配套两道守卫：亮度下限（暗区比例不可靠）+ 最小绝对差（防噪声抖动）。 */
#define BALL_GREEN_NEXG_MIN       3      /* nExG×100 下限（3 = 0.03） */
#define BALL_GREEN_SUM_MIN        200    /* 亮度下限 R+G+B */
#define BALL_GREEN_DELTA_MIN      2      /* G−max(R,B) 最小绝对差 */
#define BALL_GREEN_SCORE_MAX      63     /* 单像素得分上限（nExG×100 截断） */

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
/* 绿色只在上半区（y ≤ BALL_MARKER_Y_MAX）累积与识别：角标在赛道
 * 最远处，新出现的目标必然在画面上半；画面下半（含车身角落凸出到
 * 底部 30% 屏蔽线以上的部分）一律不产生绿候选。⚠️ 推球末期角标可能
 * 沉到门限之下，导致"球贴近角标"完成判定失效——实车若出现，放宽到
 * 0.55~0.60。 */
#define BALL_MARKER_Y_MAX        0.50f
/* 时序累积：角标静止，瞬态噪声不持续。格内得分（该格 16 像素中绿色
 * 得分之和，×8 定点）按 IIR 累积，持续过线才占用——单帧 JPEG 噪声
 * 冲不过门限，淡绿真信号逐帧攒够。取代旧"格内 3/16 命中即占用"。
 * 门限 64 ↔ 持续每帧格内得分和 ≥8（如 1 像素 nExG 0.08 或 2 像素
 * 各 0.04）；单帧突发最高只到门限的 3/10，须连续约 2 帧才占用。 */
#define BALL_MARKER_ACC_NEW      3      /* IIR 新值权重（/10；旧值 7/10） */
#define BALL_MARKER_ACC_OCCUPIED 64     /* 占用门限（得分×8 定点） */
/* 提取的最少占用格数放宽（角标远端成像只有 2~3 格）、宽松形状
 * （绿色类场上无同色源，形状交给累积和区域门限过滤）。 */
#define BALL_MARKER_MIN_CELLS    2
/* 球邻域伪源剔除的尺寸上限：与球包围盒相交且 ≤ 此格数的绿候选视为
 * JPEG 镶边伪源剔除；更大的块视为真角标保留（球推到角标旁时角标
 * 成像远大于此，完成判定不受影响）。 */
#define BALL_MARKER_FRINGE_MAX_CELLS 6

/* ---- 检测区域 ----
 * 画面底部此百分比【不参与找球识别】：云台 35° 抬起时车头/车身自身
 * 占据画面下方约 30%（2026-09-03 实测修正，曾按 20% 低估），其颜色
 * （棕/暗）会污染球分类。球实际可见区 y ∈ [0, 0.70]——对准/推球的
 * 近距判定（0.65）必须留在此边界之内，否则车会顶着球走。 */
#define BALL_DETECT_Y_BOTTOM_PCT 30

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
