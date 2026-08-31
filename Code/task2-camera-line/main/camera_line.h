#pragma once
#include <stdint.h>
#include <stdbool.h>

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

/* 摄像头流参数（ESP32-S3 USB Full Speed 带宽限制）
 * 首选 320x240；摄像头不支持时自动协商回退（640x480 等），
 * 检测与预览均按实际分辨率自适应 */
#define CAM_FRAME_WIDTH    320
#define CAM_FRAME_HEIGHT   240
#define CAM_FRAME_FPS      30

/* 黑线检测阈值标定（现场按光照调整） */
#define LINE_BLACK_Y_MAX   80    /* Y 亮度低于此值判为黑色 */
#define LINE_MIN_BLACK_PIX 30    /* 一行至少这么多黑像素才算有效线 */

/*
 * 扫描窗口（按摄像头安装几何标定）：
 * 摄像头俯角可调，调好后黑线位于画面垂直【中部】，
 * 因此只扫垂直中部窗口：上沿在画面高度 CAM_SCAN_WIN_TOP_PCT% 处、
 * 高 CAM_SCAN_WIN_HEIGHT_PCT%（默认画面 30%~70%）。
 * 若再调整俯角导致线的位置移动，改这两个宏让窗口重新罩住线即可。
 */
#define CAM_SCAN_WIN_TOP_PCT     30   /* 窗口上沿：画面高度 30% 处 */
#define CAM_SCAN_WIN_HEIGHT_PCT  40   /* 窗口高度：画面高度的 40% */

/* 巡线结果快照 */
typedef struct {
    bool has_line;        /* 是否检测到黑线 */
    bool stable;          /* has_line 已连续多帧一致；见线还满足姿态稳定等待 */
    float error;          /* 质心水平误差 [-1,+1]，左负右正，0=居中 */
    uint16_t black_rows;  /* 检出黑线的行数（诊断用） */
    uint16_t wide_black_rows; /* 黑色覆盖整行 >= 65% 的行数（T 线判定用） */
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
