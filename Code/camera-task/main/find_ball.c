#include "find_ball.h"
#include "gimbal.h"
#include "motion.h"
#include "kick.h"
#include "esp_timer.h"
#include <stdio.h>

/* ============================================================
 * 找球/踢球状态机实现
 *
 * 每个状态的行为（详见 find_ball.h 顶部注释）：
 *   SEARCH   : 云台 pan 往复扫视，每步停留 dwell 时间，找球
 *   TRACK    : 云台跟踪，让球回到画面中心（x≈0.5, y≈0.5）
 *   ALIGN    : 车原地旋转，直到云台 pan 回到正前方
 *   APPROACH : 车接近 + x_ratio 差速修正（摄像头校准闭环），
 *              球面积达阈值 → 停车踢球
 *   KICK     : 踢球动作，踢后确认球是否消失
 *
 * 电机全部走 motion_* 抽象接口（当前为占位打印）。
 * ============================================================ */

static find_ball_params_t s_p;
static find_state_t s_state = FIND_STATE_SEARCH;
static int s_search_dir = 1;          /* 扫视方向：+1 右扫 / -1 左扫 */
static int64_t s_tick_start = 0;      /* 状态停留计时 */
static int s_kick_retry = 0;
static int s_scan_step = 0;

/* ---------- 状态名表 ---------- */
static const char *s_state_name[] = {
    "SEARCH", "TRACK", "ALIGN", "APPROACH", "KICK", "KICK_WAIT",
};

const char *find_ball_state_name(find_state_t s)
{
    if ((int)s < 0 || (int)s >= (int)(sizeof(s_state_name)/sizeof(s_state_name[0])))
        return "?";
    return s_state_name[s];
}

find_state_t find_ball_get_state(void)
{
    return s_state;
}

find_ball_params_t find_ball_default_params(void)
{
    find_ball_params_t p = {
        /* SEARCH（云台机械限位：pan -120°~+120°，tilt -30°~+90°） */
        .search_pan_min = -100,
        .search_pan_max =  100,
        .search_step = 10,
        .search_dwell_ms = 150,
        .search_tilt = 20,          /* 扫视时略向下看（+20° 仰角内） */
        /* TRACK */
        .track_center_min = 0.40f,
        .track_center_max = 0.60f,
        .track_step = 5,
        /* ALIGN（0°=正前方） */
        .align_target_pan = 0,
        .align_rotate_speed = 40,
        /* APPROACH */
        .approach_target_area = 0.15f,
        .approach_speed = 50,
        .approach_diff_gain = 30,
        /* KICK */
        .kick_retry_max = 3,
    };
    return p;
}

void find_ball_init(int pan_pin, int tilt_pin, int kick_pin)
{
    s_p = find_ball_default_params();
    gimbal_init(pan_pin, tilt_pin);
    kick_init(kick_pin);
    motion_init();

    s_state = FIND_STATE_SEARCH;
    s_search_dir = 1;
    s_kick_retry = 0;
    s_scan_step = 0;
    gimbal_set_pan(s_p.search_pan_min);
    gimbal_set_tilt(s_p.search_tilt);
    s_tick_start = esp_timer_get_time();

    printf("[find_ball] 状态机初始化完成，进入 SEARCH\n");
}

/* 是否在画面中心附近（x 与 y 都接近 0.5） */
static bool ball_centered(const ball_detect_result_t *res)
{
    return (res->x_ratio >= s_p.track_center_min &&
            res->x_ratio <= s_p.track_center_max);
}

/* ---------- SEARCH：云台往复扫视 ---------- */
static void state_search(const ball_detect_result_t *res)
{
    if (res->detected) {
        printf("[find_ball] SEARCH → 发现球，进入 TRACK\n");
        s_state = FIND_STATE_TRACK;
        return;
    }

    /* 每步停留 dwell 后移动一步 */
    if (esp_timer_get_time() - s_tick_start >= s_p.search_dwell_ms * 1000) {
        s_tick_start = esp_timer_get_time();
        int cur = gimbal_get_pan() + s_search_dir * s_p.search_step;
        if (cur > s_p.search_pan_max) {
            s_search_dir = -1;
            cur = s_p.search_pan_max;
        } else if (cur < s_p.search_pan_min) {
            s_search_dir = 1;
            cur = s_p.search_pan_min;
        }
        gimbal_set_pan(cur);
        s_scan_step++;
        if (s_scan_step % 10 == 0) {
            printf("[find_ball] SEARCH 扫视 pan=%d\n", cur);
        }
    }
}

/* ---------- TRACK：云台跟踪让球居中 ---------- */
static void state_track(const ball_detect_result_t *res)
{
    if (!res->detected) {
        printf("[find_ball] TRACK → 跟丢，回 SEARCH\n");
        s_state = FIND_STATE_SEARCH;
        return;
    }

    /* 用 x_ratio 修正 pan（球偏左→云台左转） */
    int pan = gimbal_get_pan();
    if (res->x_ratio < s_p.track_center_min) {
        pan -= s_p.track_step;
    } else if (res->x_ratio > s_p.track_center_max) {
        pan += s_p.track_step;
    }
    gimbal_set_pan(pan);

    /* y 方向修正 tilt（新坐标系：tilt 正=向上仰，负=向下俯） */
    int tilt = gimbal_get_tilt();
    if (res->y_ratio < 0.40f) {
        tilt += s_p.track_step;    /* 球在画面上方 → 云台上仰 */
    } else if (res->y_ratio > 0.60f) {
        tilt -= s_p.track_step;    /* 球在画面下方 → 云台下俯 */
    }
    gimbal_set_tilt(tilt);

    /* 球已居中 → 车对位 */
    if (ball_centered(res)) {
        printf("[find_ball] TRACK → 球已居中(pan=%d)，进入 ALIGN\n", gimbal_get_pan());
        s_state = FIND_STATE_ALIGN;
    }
}

/* ---------- ALIGN：车原地旋转，云台角度回到正前方 ---------- */
static void state_align(const ball_detect_result_t *res)
{
    int pan = gimbal_get_pan();
    int err = pan - s_p.align_target_pan;   /* >0 球在右 */

    if (err > 5) {
        motion_rotate(-s_p.align_rotate_speed);   /* 球在右 → 顺时针转 */
    } else if (err < -5) {
        motion_rotate(s_p.align_rotate_speed);    /* 球在左 → 逆时针转 */
    } else {
        motion_stop();
        printf("[find_ball] ALIGN → 车头对准球(pan=%d)，进入 APPROACH\n", pan);
        gimbal_set_pan(s_p.align_target_pan);     /* 云台回正 */
        s_state = FIND_STATE_APPROACH;
    }
}

/* ---------- APPROACH：摄像头校准闭环接近 ---------- */
static void state_approach(const ball_detect_result_t *res)
{
    if (!res->detected) {
        printf("[find_ball] APPROACH → 丢球，回 SEARCH\n");
        motion_stop();
        s_state = FIND_STATE_SEARCH;
        return;
    }

    /* 到位判定：球足够大（够近） */
    if (res->area_ratio >= s_p.approach_target_area) {
        motion_stop();
        printf("[find_ball] APPROACH → 到位(area=%.3f)，踢球\n", res->area_ratio);
        s_state = FIND_STATE_KICK;
        return;
    }

    /* 横向校准：x_ratio 差速修正（摄像头反馈闭环） */
    int diff = 0;
    if (res->x_ratio < s_p.track_center_min) {
        diff = -s_p.approach_diff_gain;    /* 球偏左 → 左轮减速 */
    } else if (res->x_ratio > s_p.track_center_max) {
        diff = s_p.approach_diff_gain;     /* 球偏右 → 右轮减速 */
    }

    int base = s_p.approach_speed;
    motion_set_left_right(base - diff, base + diff);
}

/* ---------- KICK：踢球 + 确认 ---------- */
static void state_kick(const ball_detect_result_t *res)
{
    kick_execute();
    s_kick_retry = 0;
    s_state = FIND_STATE_KICK_WAIT;
    s_tick_start = esp_timer_get_time();
}

static void state_kick_wait(const ball_detect_result_t *res)
{
    /* 踢后等待 1s，若球消失认为踢出成功，回 SEARCH */
    if (esp_timer_get_time() - s_tick_start < 1000000) {
        return;
    }

    if (!res->detected || s_kick_retry >= s_p.kick_retry_max) {
        printf("[find_ball] KICK_WAIT → 踢球成功/放弃，回 SEARCH\n");
        kick_reset();
        s_kick_retry = 0;
        s_state = FIND_STATE_SEARCH;
        s_tick_start = esp_timer_get_time();
    } else {
        printf("[find_ball] KICK_WAIT → 球还在，重试踢球\n");
        s_kick_retry++;
        s_state = FIND_STATE_KICK;
    }
}

/* ---------- 主更新 ---------- */
void find_ball_update(const ball_detect_result_t *res)
{
    switch (s_state) {
    case FIND_STATE_SEARCH:      state_search(res);      break;
    case FIND_STATE_TRACK:       state_track(res);       break;
    case FIND_STATE_ALIGN:       state_align(res);       break;
    case FIND_STATE_APPROACH:    state_approach(res);    break;
    case FIND_STATE_KICK:        state_kick(res);        break;
    case FIND_STATE_KICK_WAIT:   state_kick_wait(res);   break;
    default:                     s_state = FIND_STATE_SEARCH; break;
    }
}
