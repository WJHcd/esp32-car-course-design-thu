#pragma once
#include "esp_err.h"

/* ============================================================
 * 语音识别核心（ESP-SR 离线方案）
 *
 * 流程：麦克风(UAC) → AFE 声学前端 → MultiNet 中文命令词识别（免唤醒，
 *      连续监听，自带超时窗口）→ 事件回调通知上层
 *
 * 本模块只负责“听懂”：词表（见 voice_cmd.c）一次性全量注册，
 * 识别结果统一转成 voice_evt_t 事件回调。
 * **哪个模式下哪些事件生效，由上层 app_menu 按当前模式裁决**，
 * 因此识别核心不需要关心模式切换，各功能模块可共享同一事件流。
 *
 * 全部离线识别，无需联网。需在 menuconfig 中使能唤醒词模型
 * （Component config → ESP-SR → 选择 "Hi,乐鑫"(hilexin) 等）。
 * ============================================================ */

/** @brief 语音识别事件（一个事件可对应多个词，如 后退/倒退） */
typedef enum {
    VOICE_EVT_NONE = 0,         /* 无效/未识别 */

    /* —— 功能选择菜单命令 —— */
    VOICE_EVT_SELECT_BT,        /* “蓝牙遥控” → 进入蓝牙遥控功能 */
    VOICE_EVT_SELECT_FACE,      /* “表情”     → 进入表情功能 */
    VOICE_EVT_SELECT_VOICE,     /* “语音遥控” → 进入语音遥控功能 */

    /* —— 语音遥控动作命令 —— */
    VOICE_EVT_DRIVE_FORWARD,    /* “前进” */
    VOICE_EVT_DRIVE_BACKWARD,   /* “后退”（旧版叫“倒退”，保留同义词） */
    VOICE_EVT_DRIVE_LEFT,       /* “左转” */
    VOICE_EVT_DRIVE_RIGHT,      /* “右转” */
    VOICE_EVT_DRIVE_STOP,       /* “停止” */

    /* —— 通用命令 —— */
    VOICE_EVT_BACK_SELECT,      /* “选择功能” → 任意模式返回功能选择 */

    VOICE_EVT_MAX,
} voice_evt_t;

/**
 * @brief 识别回调（在 detect 任务上下文调用，务必非阻塞；建议只做状态切换）
 * @param evt      识别到的事件（VOICE_EVT_NONE 不会被回调）
 * @param user_ctx 用户上下文
 */
typedef void (*voice_cmd_cb_t)(voice_evt_t evt, void *user_ctx);

/**
 * @brief 初始化并启动语音识别（阻塞调用，含模型加载）
 *
 * 内部创建两个任务：
 *   - voice_feed：麦克风 → AFE（固定 core 1）
 *   - voice_detect：AFE → 唤醒 → 命令识别 → 回调（固定 core 1）
 *
 * @param cb       识别到有效命令时回调（可为 NULL）
 * @param user_ctx 回调上下文
 * @return ESP_OK 成功；失败原因见日志
 */
esp_err_t voice_cmd_init(voice_cmd_cb_t cb, void *user_ctx);

/** @brief 事件转命令词文本（日志/调试用；事件有多个同义词时返回第一个） */
const char *voice_evt_to_str(voice_evt_t evt);
