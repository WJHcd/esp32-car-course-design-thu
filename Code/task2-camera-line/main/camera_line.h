#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "ball_detect.h"

/* ============================================================
 * 摄像头黑线巡线接口（任务二核心，替代任务一的红外巡线）
 *
 * 数据流：USB UVC 摄像头 MJPEG → esp_jpeg 解码 RGB888
 *         → 逐行检测黑色像素 → 计算黑线质心 → 输出归一化误差
 *
 * 输出语义与任务一 infrared_get_error() 对齐：
 *   - has_line : 本帧是否看到黑线（false = 全白丢线）
 *   - error    : 黑线质心相对画面中心的水平误差，
 *                左负右正；数值范围约 -1.0 ~ +1.0
 *   （walking.c 的 PD 控制器通过 LINE_KP 缩放适配该量纲）
 * ============================================================ */

/* ============================================================
 * 扫描窗口与二值化标定区（2026-08-31 按红外等价位置标定）
 * 现场调试只改下面 5 个宏，改完重新编译烧录。
 *
 * 设计对齐任务一红外巡线（LQ-R4CHV 四路，误差公式
 * E = (-3a - b + c + 3d) / count，a/b/c/d 为左外→右外）：
 *   - 扫描带中心 60% ≈ 红外阵列的安装高度（实测对齐）；
 *   - 扫描宽度 40% ≈ 红外阵列的物理宽度（约 40% 画面宽）；
 *   - 窗口内逐列视作虚拟探头，误差 = 命中列位置的归一化均值
 *     （红外公式的连续推广），|error|=1 对应红外最外侧探头处。
 * ============================================================ */

/* 扫描位置：扫描带中心距画面【底部】的高度百分比（实测 50 ≈ 红外位置） */
#define CAM_SCAN_CENTER_FROM_BOTTOM_PCT   60
/* 扫描带高度：以扫描中心为基准的总高度百分比（模拟红外阵列的安装余量） */
#define CAM_SCAN_BAND_HEIGHT_PCT           8
/* 扫描宽度：画面宽度的百分比，水平居中（现场标定 40→60：
 * 扩窗让旋转找线时出口线更早进入视野；注意赛道旁线也更易入窗，
 * 若出现误跟踪旁线/误差跳变，回 40~50） */
#define CAM_SCAN_WIDTH_PCT                60
/* 二值化阈值：rgb2gray 后灰度低于此值判黑（0~255，
 * 对应 MATLAB imbinarize(I,T) 的全局阈值用法；光照变强适当调大） */
#define CAM_BIN_THRESHOLD                 120
/* 见线最少黑色列数：窗口内判黑列少于此值视为丢线（"至少一路见线"的相机版） */
#define CAM_LINE_MIN_COLUMNS               3
/* 宽黑行判定：一行内黑色覆盖窗口宽度的百分比（终点 T 线/宽黑区用）。
 * 2026-09-01 现场标定 65→45：原 65% 按 40% 窗宽标定（横杆恰好压满窗口）；
 * 扫描窗加宽到 60% 后横杆只占窗口约 2/3，65% 永远凑不满，避障后终点
 * T 线停不住。45%×60% 窗 ≈ 原 65%×40% 窗的绝对覆盖要求（≈85 像素）。
 * 若锐角顶点又开始误判成 T 线停车，往 50~55 回调；仍停不住则降到 40。
 * 【实车观察 2026-09-01】横杆本身物理长度可能不足以在加宽后的窗口内
 * 凑满宽黑行——若降到 40% 仍停不住，应实测横杆宽度与扫描窗的相对
 * 尺寸，考虑回调扫描窗宽度（40~50），而不是继续降阈值（过低会让
 * 锐角顶点宽黑区误判成 T 线）。 */
#define CAM_WIDE_ROW_PCT                  45
/* 水平镜像：1 = 整帧左右翻转后再检测/显示。摄像头水平装反（实车转向
 * 方向与预期相反）时置 1；翻转在解码后统一做，检测与两路预览自动一致 */
#define CAM_FLIP_X                          1

/*
 * 摄像头流参数（ESP32-S3 USB Full Speed 带宽限制）
 * 首选 320x240；摄像头不支持时自动协商回退（640x480 等），
 * 检测与预览均按实际分辨率自适应 */
#define CAM_FRAME_WIDTH    320
#define CAM_FRAME_HEIGHT   240
#define CAM_FRAME_FPS      30

/* 巡线结果快照 */
typedef struct {
    bool has_line;        /* 是否检测到黑线 */
    bool stable;          /* has_line 已连续多帧一致；见线还满足姿态稳定等待 */
    float error;          /* 质心水平误差 [-1,+1]，左负右正，0=扫描窗中心 */
    uint16_t black_rows;  /* 扫描带内检出黑线的行数（诊断用） */
    uint16_t wide_black_rows; /* 黑色覆盖窗口 >= CAM_WIDE_ROW_PCT% 的行数（T 线用） */
    uint8_t line_pattern; /* 仿真红外 a/b/c/d 图案：bit3~bit0 = a(左外)~d(右外) */
    uint32_t frame_seq;   /* 每成功解码一帧递增一次，避免主循环重复消费旧帧 */
} camera_line_result_t;

/**
 * @brief 初始化摄像头巡线模块（USB Host + UVC + 处理任务）
 * @return true=初始化流程已启动（异步等待摄像头接入）
 */
bool camera_line_init(void);

/**
 * @brief 获取最近一帧的巡线结果（非阻塞）
 * @param out 输出结果；摄像头未就绪时 has_line=false
 */
void camera_line_get(camera_line_result_t *out);

/**
 * @brief 获取最近一帧的找球任务三色识别结果（黑球/蓝球/红角标）
 * @param out 输出结果；每帧覆盖，摄像头未就绪时全零
 */
void camera_line_get_ball(ball_field_result_t *out);

/** @brief 诊断：帧回调累计触发次数 */
uint32_t camera_line_frame_cb_count(void);

/**
 * @brief 摄像头是否已开始出帧
 */
bool camera_line_ready(void);

/**
 * @brief 摄像头帧回调类型：每解码出一帧 RGB888 图像调用一次
 * @param rgb888 RGB888 像素数据（w*h*3 字节，仅在回调期间有效）
 * @param width  图像宽度（像素）
 * @param height 图像高度（像素）
 */
typedef void (*camera_line_frame_cb_t)(const uint8_t *rgb888, int width, int height);

/**
 * @brief 注册帧回调（传 NULL 取消）。
 * 用于把画面转发给外部显示（如 TFT 摄像头预览）。
 * 注意：回调在图像处理任务上下文中执行，内部勿做阻塞耗时的操作。
 */
void camera_line_set_frame_cb(camera_line_frame_cb_t cb);

/**
 * @brief 开/关处理任务的周期统计日志（500ms 一条）
 * ASCII 预览等需要干净串口输出的场景可关闭。
 */
void camera_line_set_stats_log(bool enable);

/**
 * @brief 摄像头巡线结果是否"稳定可信"（任务二新增）
 *
 * 摄像头帧率约 10~15fps（JPEG 软解码），单帧结果有噪声；
 * 且线只占画面中部窄窄一条，车一动画面变化大。
 * 上层动作（转向/避障）应等本函数返回 true 再执行：
 *   - has_line 需连续 CAM_STABLE_SAMPLES 帧一致才翻转状态
 *   - 首次见线后额外等待 CAM_SETTLE_MS，让画面稳定
 */
bool camera_line_stable(void);
