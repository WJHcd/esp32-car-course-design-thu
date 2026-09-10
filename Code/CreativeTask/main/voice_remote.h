#pragma once
#include <stdbool.h>
#include "voice_cmd.h"

/* ============================================================
 * 语音遥控功能（已实现，真实电机驱动）
 *
 * 进入语音遥控模式后，语音直接指挥小车：
 *
 *   “前进”/“后退” —— 前向分量直行（持续，直到新命令/停止/返回菜单）
 *   “左转”/“右转” —— 旋转分量原地转（持续，同上）
 *   “停止”         —— 全停
 *   “选择功能”      —— 由 app_menu 接管：返回功能选择（先刹停）
 *
 * 动作通过 motion_drive(fwd, turn, strafe) 下发，速度参数是
 * 相对权重（见 voice_remote.c 顶部宏），实际轮速由 motion.c
 * 归一化到实车标定值；如需调整请改宏并实车验证。
 * 进入模式时 motion_enable(true)，退出时刹停并 disable——
 * 语音遥控模式外电机永不使能。
 * ============================================================ */

/**
 * @brief 初始化语音遥控模块（记录状态）
 */
void voice_remote_init(void);

/**
 * @brief 进入语音遥控模式（app_menu 切到 APP_MODE_VOICE 时调用）
 *        内部使能电机并刹停，等待第一条命令
 */
void voice_remote_enter(void);

/**
 * @brief 退出语音遥控模式（返回功能选择时调用）：刹停并关闭电机使能
 */
void voice_remote_exit(void);

/**
 * @brief 执行语音遥控动作事件
 *
 * 仅处理动作类事件（前进/后退/左转/右转/停止），
 * 其余事件（菜单选择词等）在此忽略并提示，由 app_menu 负责路由。
 *
 * @param evt 识别事件（voice_cmd 回调上下文调用，非阻塞）
 */
void voice_remote_execute(voice_evt_t evt);

/** @brief 当前是否处于语音遥控模式（供测试判断） */
bool voice_remote_is_active(void);
