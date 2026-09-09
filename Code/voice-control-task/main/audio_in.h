#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* ============================================================
 * 音频输入抽象层
 *
 * 为 ESP-SR 语音识别提供 **16kHz / 16bit / 单声道** 的音频流。
 *
 * 当前实现：audio_in_uac.c —— JQ-CAM12-720P 三合一摄像头板载麦克风，
 *   音频与视频共用 USB 线（UVC+UAC 复合设备），USB Host isoc 捕获。
 *   接入后自动起流；未接入时 audio_in_read() 返回 ESP_ERR_TIMEOUT。
 *
 * 备选实现：audio_in_i2s.c（独立 I2S 数字麦克风，如 INMP441），
 *   未参与构建；切换见 main/CMakeLists.txt 注释。两种实现保持本接口不变，
 *   上层（voice_cmd）无需修改。
 * ============================================================ */

/**
 * @brief 初始化音频输入（当前实现：USB UAC 麦克风）
 * @return ESP_OK 成功
 */
esp_err_t audio_in_init(void);

/**
 * @brief 阻塞读取 samples 个采样（16bit 有符号，单声道）
 * @param buf      输出缓冲（至少 samples 个 int16_t）
 * @param samples  期望读取的采样数
 * @return ESP_OK 成功（不足部分补零）；ESP_ERR_TIMEOUT=设备未接入（上层延时重试）
 */
esp_err_t audio_in_read(int16_t *buf, size_t samples);

/**
 * @brief 当前是否有音频流在持续输出（状态屏用）
 * @return true=麦克风出流中
 */
bool audio_in_is_streaming(void);
