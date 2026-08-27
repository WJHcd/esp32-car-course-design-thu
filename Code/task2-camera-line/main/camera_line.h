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

/* 摄像头流参数（ESP32-S3 USB Full Speed 带宽限制） */
#define CAM_FRAME_WIDTH    320
#define CAM_FRAME_HEIGHT   240
#define CAM_FRAME_FPS      30
#define CAM_FRAME_FORMAT   UVC_FRAME_FORMAT_MJPEG

/* 黑线检测阈值标定（现场按光照调整） */
#define LINE_BLACK_Y_MAX   80    /* Y 亮度低于此值判为黑色 */
#define LINE_MIN_BLACK_PIX 30    /* 一行至少这么多黑像素才算有效线 */

/* 巡线结果快照 */
typedef struct {
    bool has_line;        /* 是否检测到黑线 */
    float error;          /* 质心水平误差 [-1,+1]，左负右正，0=居中 */
    uint16_t black_rows;  /* 检出黑线的行数（诊断用） */
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
