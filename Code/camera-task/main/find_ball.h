#pragma once
#include <stdbool.h>
#include "ball_detect.h"

/* ============================================================
 * 找球/踢球状态机（科目三核心控制逻辑）
 *
 * 状态流：
 *   SEARCH（云台扫视找球）
 *   → TRACK（云台跟踪让球居中）
 *   → ALIGN（车对位：云台角度回正前方）
 *   → APPROACH（摄像头校准下接近球）
 *   → KICK（踢球）
 *   → 回 SEARCH
 *
 * 依赖：ball_detect（视觉）、gimbal（云台）、motion（电机接口）、kick（踢球）
 * 由外部周期调用 find_ball_update()（建议每帧/20ms）。
 * ============================================================ */

typedef enum {
    FIND_STATE_SEARCH = 0,   /* 云台扫视找球 */
    FIND_STATE_TRACK,        /* 云台跟踪球 */
    FIND_STATE_ALIGN,        /* 车对位 */
    FIND_STATE_APPROACH,     /* 摄像头校准下接近 */
    FIND_STATE_KICK,         /* 踢球 */
    FIND_STATE_KICK_WAIT,    /* 踢后确认 */
} find_state_t;

/** @brief 状态机运行参数（可现场调） */
typedef struct {
    /* SEARCH */
    int   search_pan_min;      /* 扫视左限位（度） */
    int   search_pan_max;      /* 扫视右限位（度） */
    int   search_step;         /* 每步角度增量（度） */
    int   search_dwell_ms;     /* 每步停留时间（ms） */
    int   search_tilt;         /* 扫视时固定俯角（度） */

    /* TRACK */
    float track_center_min;    /* 画面中心判定下界（x_ratio） */
    float track_center_max;    /* 画面中心判定上界（x_ratio） */
    int   track_step;          /* 每帧云台修正角度（度） */

    /* ALIGN */
    int   align_target_pan;    /* 车对位后云台应回的角度（90=正前） */
    int   align_rotate_speed;  /* 对位旋转速度 */

    /* APPROACH */
    float approach_target_area;/* 球面积占比阈值（到位停车） */
    int   approach_speed;      /* 接近速度 */
    int   approach_diff_gain;  /* 差速修正增益 */

    /* KICK */
    int   kick_retry_max;      /* 踢后未成功的重试次数 */
} find_ball_params_t;

/** @brief 获取默认参数（现场标定后可直接改结构体） */
find_ball_params_t find_ball_default_params(void);

/**
 * @brief 初始化找球状态机
 * @param pan_pin   云台左右舵机 GPIO（0=未接）
 * @param tilt_pin  云台上下舵机 GPIO（0=未接）
 * @param kick_pin  踢球舵机 GPIO（0=未接）
 */
void find_ball_init(int pan_pin, int tilt_pin, int kick_pin);

/**
 * @brief 周期调用（每帧/20ms），根据视觉结果驱动云台/电机/踢球
 * @param res  当前帧的找球结果（来自 ball_detect_process_rgb888）
 */
void find_ball_update(const ball_detect_result_t *res);

/** @brief 读取当前状态 */
find_state_t find_ball_get_state(void);

/** @brief 状态名（打印用） */
const char *find_ball_state_name(find_state_t s);
