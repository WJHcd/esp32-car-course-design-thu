#pragma once
#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 小球颜色识别接口（科目三核心）
 *
 * 输入：一帧 RGB888 图像
 * 输出：球是否在画面中 + 球的具体方位信息
 *
 * 供上层（运动学/状态机）使用的关键字段：
 *   - detected + color   : 画面中是否有球、什么颜色
 *   - x_ratio            : 球心水平方位 [0,1]，0.5=正中（差速转向用）
 *   - y_ratio            : 球心垂直方位 [0,1]，0=顶部
 *   - area_ratio         : 球面积/画面面积 [0,1]，越大球越近（前进/停止用）
 * ============================================================ */

/* 球颜色 */
typedef enum {
    BALL_COLOR_UNKNOWN = 0,
    BALL_COLOR_RED,     /* 红色球 */
    BALL_COLOR_WHITE,   /* 白色球 */
} ball_color_t;

/* 找球结果：球是否在画面中 + 方位信息 */
typedef struct {
    bool detected;         /* 画面中是否检测到球 */
    ball_color_t color;    /* 检测到的颜色（红/白） */
    float x_ratio;         /* 球心 X 归一化 [0,1]，0.5 = 画面正中 */
    float y_ratio;         /* 球心 Y 归一化 [0,1]，0 = 画面顶部 */
    float area_ratio;      /* 球像素面积 / 画面面积 [0,1]，越大越近 */
    uint16_t width_px;     /* 球包围盒宽度（像素） */
    uint16_t height_px;    /* 球包围盒高度（像素） */
    uint32_t pixel_count;  /* 球像素总数 */
} ball_detect_result_t;

/* ============================================================
 * 颜色阈值标定参数（现场按实际球的颜色调整）
 * ============================================================ */
#define BALL_RED_R_MIN      100   /* 红色：R 通道最小值 */
#define BALL_RED_RG_DELTA   40    /* 红色：R 需比 G 高出的差值 */
#define BALL_RED_RB_DELTA   40    /* 红色：R 需比 B 高出的差值 */
#define BALL_WHITE_MIN      160   /* 白色：三通道最低值 */
#define BALL_WHITE_TOL      30    /* 白色：三通道最大差值（接近白） */
#define BALL_MIN_PIXELS     20    /* 最少像素数（滤除噪声小区域） */
#define BALL_MAX_AREA_RATIO 0.6f  /* 最大面积占比（滤除全屏误判） */

/**
 * @brief 初始化找球模块
 * @param width  图像宽度（像素）
 * @param height 图像高度（像素）
 */
void ball_detect_init(int width, int height);

/**
 * @brief 对一帧 RGB888 图像执行颜色识别，输出球方位结果
 * @param rgb   RGB888 像素数据（长度 width*height*3）
 * @param out   输出结果（detected=false 时其他字段无效）
 */
void ball_detect_process_rgb888(const uint8_t *rgb, ball_detect_result_t *out);

/**
 * @brief 获取当前标定的颜色阈值（调试/打印用）
 */
void ball_detect_get_thresholds(uint8_t *red_r_min, uint8_t *red_rg_delta,
                                uint8_t *red_rb_delta, uint8_t *white_min,
                                uint8_t *white_tol, uint32_t *min_pixels);
