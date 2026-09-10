#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

/* ============================================================
 * USB 音频（UAC，摄像头三合一）：麦克风输入 + 喇叭输出
 *
 * 麦克风（RX）：16k/1ch/16bit 出流，供 ESP-SR 语音识别 feed
 *   （接口保持与原 audio_in.h 一致：audio_in_init/read/is_streaming）
 * 喇叭（TX）：攻城槌模式播放提示音。设备仅有的喇叭在摄像头上，
 *   与视频共用 USB 线；流参数自动协商（16k/48k × 单/双声道，
 *   选设备支持的组合），单声道 WAV 由软件升为立体声。
 *
 * ⚠️ usb_host_install 由 main 的 camera_task_start() 先行完成，
 *    本模块只装 uac_host 驱动（若未装 usb host 会自动补装）。
 * ============================================================ */

/** 提示音编号（对应嵌入的 WAV 资源） */
typedef enum {
    AUDIO_SND_SEARCH = 0,   /* 找目标阶段音效 */
    AUDIO_SND_ATTACK,       /* 撞击阶段音效 */
    AUDIO_SND_MAX,
} audio_snd_t;

/** @brief 初始化 UAC 驱动（麦克风自动出流；喇叭在首次播放时按需启动） */
esp_err_t audio_in_init(void);

/** @brief 读麦克风 PCM（voice_cmd feed 用）；无流返回 ESP_ERR_TIMEOUT */
esp_err_t audio_in_read(int16_t *buf, size_t samples);

/** @brief 麦克风是否已出流 */
bool audio_in_is_streaming(void);

/**
 * @brief 循环播放指定提示音（在独立任务中写 UAC 喇叭，非阻塞返回）
 * @param snd AUDIO_SND_SEARCH / AUDIO_SND_ATTACK
 */
esp_err_t audio_out_play(audio_snd_t snd);

/** @brief 停止播放 */
void audio_out_stop(void);

/** @brief 是否在播放中 */
bool audio_out_is_playing(void);
