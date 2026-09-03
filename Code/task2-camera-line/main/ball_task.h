#pragma once
#include <stdint.h>
#include "camera_line.h"

/* ============================================================
 * 找球/推球任务状态机（科目三，接在"避障→巡线→1s 停车"之后）
 *
 * 任务：场地内有两个球（黑、蓝，颜色仅用于区分目标），把它们
 * 分别推到场地末端的两个红色角标处。摄像头俯仰由云台抬起获得
 * 俯视视角；水平扫视靠车身原地旋转（tilt 单舵机方案）。
 *
 * 流程：GIMBAL_UP 抬云台 → SURVEY 原地旋转扫视（双球各确认
 * 若干帧）→ AIM 对准先推的球 → PUSH 顶球推进（有红角标可见时
 * 朝角标方向推，球与角标贴近 = 到位）→ 换下一球 → 全部完成。
 *
 * 电机指令经 walking_direct_command() 下发（走正常三轴混控）。
 * ============================================================ */

typedef enum {
    BALL_TASK_GIMBAL_UP = 0,   /* 抬云台并静置 */
    BALL_TASK_SURVEY,          /* 原地旋转扫视找双球 */
    BALL_TASK_AIM,             /* 对准先推的球 */
    BALL_TASK_PUSH,            /* 顶球推进（朝可见红角标方向） */
    BALL_TASK_DONE,            /* 双球完成（或超时兜底） */
} ball_task_state_t;

/* ---- 现场标定参数 ---- */
#define BALL_TASK_TILT_SURVEY_DEG   35     /* 扫视时云台仰角 */
#define BALL_GIMBAL_DWELL_MS        600u   /* 抬云台后的静置时长 */
#define BALL_SURVEY_TURN_PCT        0   /* 诊断：暂停扫视旋转，静止盯正前方 */      /* 扫视旋转力度 */
#define BALL_SURVEY_CONFIRM         2      /* 每个球需确认的帧数 */
#define BALL_SURVEY_TIMEOUT_MS      12000u /* 扫视总超时 */
#define BALL_AIM_DEADBAND           0.08f  /* 对准死区（x_ratio 偏离中心） */
#define BALL_AIM_TIMEOUT_MS         4000u  /* 对准超时（超时重扫） */
#define BALL_PUSH_SPEED_PCT         12     /* 推球前进力度 */
#define BALL_PUSH_STEER_GAIN        8.0f   /* 对准修正：x 偏差 → 转向 % */
#define BALL_PUSH_TIMEOUT_MS        15000u /* 单球推球总超时 */
#define BALL_MARKER_ADJACENT_ERR    0.15f  /* 球心与红角标贴近 = 到位 */
#define BALL_LOST_BLIND_MS          300u   /* 近距离丢球后的盲走时长 */
#define BALL_TASK_TOTAL_MS          120000u/* 全任务兜底超时 */

/**
 * @brief 启动找球任务（抬起云台并进入 GIMBAL_UP）
 */
void ball_task_start(uint32_t now_ms);

/**
 * @brief 每控制周期调用（20ms）；内部按新帧处理识别结果并下发电机指令
 * @param cam   最近巡线结果（仅用 frame_seq 判新帧）
 * @param balls 最近一帧三色识别结果（camera_line_get_ball 的输出）
 */
void ball_task_update(const camera_line_result_t *cam,
                      const ball_field_result_t *balls, uint32_t now_ms);

/**
 * @brief 双球任务是否已结束（含超时兜底），walking 据此进 FINISH_STOP
 */
bool ball_task_is_done(void);

/** @brief walking_reset 联动复位 */
void ball_task_reset(uint32_t now_ms);

/** @brief 当前状态名（串口显示用） */
const char *ball_task_state_name(void);
