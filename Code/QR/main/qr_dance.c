#include "qr_dance.h"
#include "display.h"
#include "motion.h"
#include "esp_timer.h"
#include <stdio.h>
#include <string.h>

/* ============================================================
 * 表情/舞蹈状态机实现
 *
 * WAIT_QR：等新解码结果；内容与上次触发不同且命中组合表 → DANCE。
 * DANCE  ：转圈(spin) → 扭动(wiggle，左右交替) → 下一循环，
 *          到 sequence_loop 次后自动停，回 WAIT_QR。
 * 计时全部用 esp_timer，与调用频率无关。
 * ============================================================ */

#define ROUTINE_COUNT 3

static const char *s_contents[ROUTINE_COUNT] = { "1", "2", "3" };
static const dance_routine_t s_routines[ROUTINE_COUNT] = {
    /* QR "1"：笑脸 + 顺时针转圈 + 扭动 */
    { .face_id = 0, .spin_speed = 40, .spin_ms = 2000, .spin_dir = +1,
      .wiggle_speed = 30, .wiggle_ms = 300, .wiggle_repeat = 6, .sequence_loop = 2 },
    /* QR "2"：哭脸 + 逆时针转圈 + 慢扭动 */
    { .face_id = 1, .spin_speed = 40, .spin_ms = 1500, .spin_dir = -1,
      .wiggle_speed = 30, .wiggle_ms = 400, .wiggle_repeat = 4, .sequence_loop = 2 },
    /* QR "3"：惊讶 + 快转 + 快扭动 */
    { .face_id = 2, .spin_speed = 50, .spin_ms = 2500, .spin_dir = +1,
      .wiggle_speed = 40, .wiggle_ms = 250, .wiggle_repeat = 8, .sequence_loop = 1 },
};

static dance_state_t s_state = DANCE_STATE_WAIT_QR;
static char s_last_content[QR_TEXT_MAX] = {0};
static int  s_routine_idx = 0;
static int64_t s_phase_start = 0;
static int  s_dance_phase = 0;   /* 0=转圈, 1=扭动 */
static int  s_wiggle_seg = 0;
static int  s_seq_loop = 0;

int qr_dance_lookup(const char *content)
{
    if (!content) return -1;
    for (int i = 0; i < ROUTINE_COUNT; i++) {
        if (strcmp(content, s_contents[i]) == 0) return i;
    }
    return -1;
}

const char *qr_dance_state_name(void)
{
    return (s_state == DANCE_STATE_ACTIVE) ? "DANCE" : "WAIT_QR";
}

dance_state_t qr_dance_get_state(void)
{
    return s_state;
}

bool qr_dance_is_active(void)
{
    return s_state == DANCE_STATE_ACTIVE;
}

static void enter_wait(void)
{
    motion_stop();
    display_idle();
    s_state = DANCE_STATE_WAIT_QR;
}

void qr_dance_init(void)
{
    s_last_content[0] = '\0';
    s_routine_idx = 0;
    s_dance_phase = 0;
    s_wiggle_seg = 0;
    s_seq_loop = 0;
    enter_wait();
    printf("[dance] 初始化完成，等待二维码\n");
}

static void start_dance(int idx)
{
    s_routine_idx = idx;
    s_dance_phase = 0;
    s_wiggle_seg = 0;
    s_seq_loop = 0;
    s_phase_start = esp_timer_get_time();
    display_face(s_routines[idx].face_id);
    s_state = DANCE_STATE_ACTIVE;
    printf("[dance] DANCE 开始 face=%d spin=%dms wiggle=%dms x%d loop=%d\n",
           s_routines[idx].face_id, s_routines[idx].spin_ms,
           s_routines[idx].wiggle_ms, s_routines[idx].wiggle_repeat,
           s_routines[idx].sequence_loop);
}

/* 推进舞蹈：转圈 → 扭动（左右各 wiggle_ms，来回 repeat 次）→ 下一循环 */
static void dance_tick(void)
{
    const dance_routine_t *r = &s_routines[s_routine_idx];
    const int64_t now = esp_timer_get_time();
    const int64_t el = now - s_phase_start;

    if (s_dance_phase == 0) {
        /* 转圈 */
        if (el >= (int64_t)r->spin_ms * 1000) {
            s_dance_phase = 1;
            s_wiggle_seg = 0;
            s_phase_start = now;
        } else {
            motion_rotate(r->spin_dir * r->spin_speed);
        }
        return;
    }

    /* 扭动：每 wiggle_ms 换一次方向，一个来回 = 2 段 */
    const int seg = (int)(el / ((int64_t)r->wiggle_ms * 1000));
    if (seg >= r->wiggle_repeat * 2) {
        s_seq_loop++;
        if (s_seq_loop >= r->sequence_loop) {
            printf("[dance] DANCE 结束\n");
            enter_wait();
            return;
        }
        s_dance_phase = 0;
        s_phase_start = now;
        return;
    }
    const int dir = (seg % 2 == 0) ? +1 : -1;
    motion_rotate(dir * r->wiggle_speed);
}

void qr_dance_update(const qr_result_t *qr)
{
    if (s_state == DANCE_STATE_ACTIVE) {
        dance_tick();
        return;
    }

    /* WAIT_QR：只有拿到新解码结果才处理 */
    if (!qr || !qr->detected) return;

    if (strcmp(qr->text, s_last_content) == 0) {
        return;   /* 同一码防重 */
    }
    strncpy(s_last_content, qr->text, QR_TEXT_MAX - 1);
    s_last_content[QR_TEXT_MAX - 1] = '\0';

    const int idx = qr_dance_lookup(qr->text);
    if (idx >= 0) {
        printf("[dance] QR:\"%s\" → 触发\n", qr->text);
        start_dance(idx);
    } else {
        printf("[dance] QR:\"%s\" 未匹配组合表，忽略\n", qr->text);
    }
}
