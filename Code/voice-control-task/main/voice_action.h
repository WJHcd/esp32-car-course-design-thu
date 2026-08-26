#pragma once
#include "voice_cmd.h"

/* ============================================================
 * 语音动作执行器：命令 → 小车运动（对接 motion 接口）
 *
 * 语义：
 *   前进/倒退/左转/右转 —— 持续执行，直到新命令或“停止”
 *   调头               —— 原地转 180° 后自动停（时长可标定）
 *   停止               —— 全停
 * ============================================================ */

/**
 * @brief 初始化动作执行器（创建独立任务 + 命令队列）
 *        内部会调用 motion_init()。
 */
void voice_action_init(void);

/**
 * @brief 执行语音命令（voice_cmd 回调入口，非阻塞，投递队列）
 * @param cmd   命令
 * @param ctx   未使用
 */
void voice_action_execute(voice_cmd_t cmd, void *ctx);
