#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

/* ============================================================
 * 音频输入抽象层
 *
 * 为 ESP-SR 语音识别提供 **16kHz / 16bit / 单声道** 的音频流。
 *
 * 当前实现：audio_in_i2s.c（I2S 数字麦克风，如 INMP441）
 * 若改用摄像头 USB 麦克风（UAC），新增 audio_in_uac.c 并保持本接口不变，
 * 上层（voice_cmd）无需修改。
 * ============================================================ */

/**
 * @brief 初始化音频输入（I2S 麦克风）
 * @return ESP_OK 成功
 */
esp_err_t audio_in_init(void);

/**
 * @brief 阻塞读取 samples 个采样（16bit 有符号，单声道）
 * @param buf      输出缓冲（至少 samples 个 int16_t）
 * @param samples  期望读取的采样数
 * @return ESP_OK 成功（不足部分补零）
 */
esp_err_t audio_in_read(int16_t *buf, size_t samples);
