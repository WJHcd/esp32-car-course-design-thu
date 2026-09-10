#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "qr_detect.h"

/* ============================================================
 * 摄像头采集 + 二维码识别后台任务（移植自 Code/QR 工程）
 *
 * UVC(MJPEG) → esp_jpeg 解码 → 灰度 → quirc（300ms 节流）。
 * 识别结果持续更新，供攻城槌模式查询（是否看到 "1"/"2" 及位置）。
 * ============================================================ */

/**
 * @brief 启动摄像头管线（含 usb_host_install / uvc_host 驱动 / 处理任务）
 *        ⚠️ 必须最先调用（其他 USB 模块依赖它装好 usb host）
 */
esp_err_t camera_task_start(void);

/**
 * @brief 取最近一次二维码识别结果
 * @param out     结果拷贝（text/x_ratio/y_ratio/count）
 * @param age_ms  输出：该结果距今多少毫秒（从未识别过返回 false）
 * @return false = 尚无任何识别结果
 */
bool camera_qr_get(qr_result_t *out, int64_t *age_ms);
