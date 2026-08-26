#pragma once
#include "esp_err.h"

/* ============================================================
 * 语音识别核心（ESP-SR 离线方案）
 *
 * 流程：麦克风(I2S) → AFE 声学前端 → WakeNet 唤醒
 *       → MultiNet 中文命令词识别 → 回调通知上层
 *
 * 命令词：前进 / 倒退 / 左转 / 右转 / 调头 / 停止
 * 全部离线识别，无需联网。需在 menuconfig 中使能唤醒词模型
 * （Component config → ESP-SR → 选择 "Hi,乐鑫"(hilexin) 等）。
 * ============================================================ */

/** @brief 语音命令枚举（ID 与 esp_mn_commands_add 的命令 ID 一致） */
typedef enum {
    VOICE_CMD_NONE = 0,     /* 无效/未识别 */
    VOICE_CMD_FORWARD,      /* 前进 */
    VOICE_CMD_BACKWARD,     /* 倒退 */
    VOICE_CMD_LEFT,         /* 左转 */
    VOICE_CMD_RIGHT,        /* 右转 */
    VOICE_CMD_UTURN,        /* 调头 */
    VOICE_CMD_STOP,         /* 停止 */
    VOICE_CMD_MAX,
} voice_cmd_t;

/** @brief 识别回调（在 detect 任务上下文调用，务必非阻塞；建议投递队列） */
typedef void (*voice_cmd_cb_t)(voice_cmd_t cmd, void *user_ctx);

/**
 * @brief 初始化并启动语音识别（阻塞调用，含模型加载）
 *
 * 内部创建两个任务：
 *   - voice_feed：麦克风 → AFE（建议固定 core 1）
 *   - voice_detect：AFE → 唤醒 → 命令识别 → 回调
 *
 * @param cb       识别到有效命令时回调（可为 NULL）
 * @param user_ctx 回调上下文
 * @return ESP_OK 成功；失败原因见日志
 */
esp_err_t voice_cmd_init(voice_cmd_cb_t cb, void *user_ctx);

/** @brief 命令转中文名（日志/调试用） */
const char *voice_cmd_to_str(voice_cmd_t cmd);
