#pragma once
#include <stdbool.h>
#include "qr_detect.h"

/* ============================================================
 * 表情/舞蹈状态机
 *
 *   WAIT_QR ──(识别到新内容，命中组合表)──▶ DANCE ──(时长到)──▶ WAIT_QR
 *
 * 由外部在每个控制周期调用 qr_dance_update()：
 *   - 传入 NULL 或未识别到新码时，若在 DANCE 中则推进舞蹈计时；
 *   - 传入新解码结果时，在 WAIT_QR 中做"内容变化"判断并触发。
 *
 * 舞蹈 = 转圈(spin) + 扭动(wiggle) 交替，sequence_loop 次后自动停止。
 * ============================================================ */

/* 一个二维码内容对应的"表情 + 舞蹈编排" */
typedef struct {
    int  face_id;           /* display 表情编号（0=笑脸 1=哭脸 2=惊讶 3=眨眼） */
    /* 转圈 */
    int  spin_speed;        /* 转圈速度 %（正=正转，方向见 motion 标定） */
    int  spin_ms;           /* 转圈时长 ms */
    int  spin_dir;          /* +1 / -1（顺/逆） */
    /* 扭动 */
    int  wiggle_speed;      /* 扭动速度 % */
    int  wiggle_ms;         /* 单次扭动时长 ms（每次左右各占一段） */
    int  wiggle_repeat;     /* 扭动来回次数 */
    /* 整组循环 */
    int  sequence_loop;     /* "转圈+扭动"整组重复次数 */
} dance_routine_t;

typedef enum {
    DANCE_STATE_WAIT_QR = 0,
    DANCE_STATE_ACTIVE,
} dance_state_t;

/**
 * @brief 初始化状态机（显示待机表情，停在 WAIT_QR）
 */
void qr_dance_init(void);

/**
 * @brief 每周期调用
 * @param qr  本周期新解码结果；无新结果传 NULL
 */
void qr_dance_update(const qr_result_t *qr);

/** @brief 当前是否在跳舞中 */
bool qr_dance_is_active(void);

/** @brief 当前状态 */
dance_state_t qr_dance_get_state(void);

/** @brief 状态名（打印用） */
const char *qr_dance_state_name(void);

/** @brief 内容→组合表查表（供测试/调试） */
int qr_dance_lookup(const char *content);
