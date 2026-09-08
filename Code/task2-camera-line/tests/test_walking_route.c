/* Host regression: compile with -I tests/stubs -I main; no ESP firmware build. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "walking.c"

static float pwm[3];
static unsigned ball_starts;
static encoder_motion_t motion;
static uint32_t now;
static camera_line_result_t frame;
void motor_init(void) {}
void motor_enable(bool e) { (void)e; }
void motor_set_speed(int id, float value) { pwm[id] = value; }
void motor_stop_all(void) { for (int i = 0; i < 3; i++) pwm[i] = 0; }
void encoder_update(void) {}
encoder_motion_t encoder_get_motion(void) { return motion; }
void ball_task_reset(uint32_t t) { (void)t; }
void ball_task_start(uint32_t t) { (void)t; ball_starts++; }
void ball_task_update(const camera_line_result_t *c, const ball_field_result_t *b, uint32_t t)
{ (void)c; (void)b; (void)t; }
bool ball_task_is_done(void) { return false; }
bool ball_task_succeeded(void) { return false; }
void camera_line_get_observation(camera_observation_t *o) { *o = (camera_observation_t){0}; }

static void reset(void)
{
    now = 1000; frame = (camera_line_result_t){0};
    motion = (encoder_motion_t){0}; ball_starts = 0;
    walking_reset(now); walking_set_output_enabled(true);
}
static void tick(bool fresh, bool line, float error, unsigned dt, uint16_t distance)
{
    now += dt;
    if (fresh) {
        frame.frame_seq++; frame.frame_ms = now;
        frame.has_line = line; frame.error = error; frame.stable = true;
    }
    walking_update_cam(&frame, distance, now);
}
#define NEXT(line, err) tick(true, line, err, 160, ULTRASONIC_INVALID)

static void test_turn_lock(void)
{
    reset(); NEXT(true, .7f); NEXT(true, .7f);
    assert(s_turn_locked && s_corner_turn_side == 1);
    NEXT(true, -.8f); NEXT(true, -.8f);
    assert(s_corner_turn_side == 1 && s_last_command.turn_percent >= 0);
    NEXT(false, 0);
    assert(s_state == CAR_STATE_CORNER_SEARCH && s_last_command.forward_percent == 0);
    NEXT(true, -.7f); /* old line on wrong side must not restore RUN */
    assert(s_state == CAR_STATE_CORNER_SEARCH && s_last_command.turn_percent > 0);
    NEXT(true, .1f);
    assert(s_last_command.turn_percent == 0);
    tick(false, true, .1f, 20, ULTRASONIC_INVALID);
    assert(s_state == CAR_STATE_CORNER_SEARCH); /* repeated frame is not confirmation */
    NEXT(true, .1f);
    assert(s_state == CAR_STATE_CORNER_EXIT_LAUNCH && s_route_index == 0);
    NEXT(true, .1f); NEXT(true, .1f);
    NEXT(true, .1f); NEXT(true, .1f);
    assert(s_state == CAR_STATE_RUN && !s_turn_locked && s_route_index == 1);
}
static void test_camera_fallback_and_timeout(void)
{
    reset(); NEXT(true, -.7f); NEXT(true, -.7f);
    assert(s_turn_locked && s_corner_turn_side == -1 && !s_route_hints_valid);
    NEXT(false, 0);
    for (int i = 0; i < 33; i++) NEXT(false, 0);
    assert(s_state == CAR_STATE_ROUTE_HOLD && s_last_command.turn_percent == 0);
    NEXT(true, 0);
    assert(s_state == CAR_STATE_ROUTE_HOLD); /* no blind reversal after timeout */
}
static void test_route_sequence(void)
{
    reset();
    for (unsigned i = 0; i < 4; i++) {
        const int side = s_route_turns[i];
        NEXT(true, side * .7f); NEXT(true, side * .7f);
        NEXT(false, 0);
        assert(s_corner_turn_side == side && s_turn_locked);
        NEXT(true, .1f); NEXT(true, .1f);
        assert(s_state == CAR_STATE_CORNER_EXIT_LAUNCH);
        for (int n = 0; n < 5; n++) NEXT(true, .1f);
        assert(s_route_index == i + 1 && s_route_hints_valid && !s_turn_locked);
        if (i == 0) { /* shallow left kink must not consume a major-corner hint */
            NEXT(true, -.3f); NEXT(true, -.3f); NEXT(true, 0);
            assert(s_route_index == 1);
        }
    }
}
static void test_raw_frames_and_filter(void)
{
    reset(); NEXT(true, .2f);
    float filtered = s_filtered_error;
    for (int i = 0; i < 6; i++) tick(false, true, .2f, 20, ULTRASONIC_INVALID);
    assert(s_filtered_error == filtered);
    now += 160; frame.frame_seq++; frame.frame_ms = now;
    frame.stable = false; frame.has_line = false;
    walking_update_cam(&frame, ULTRASONIC_INVALID, now);
    assert(!s_cam.has_line && s_last_command.forward_percent == 0);
    tick(false, false, 0, 701, ULTRASONIC_INVALID);
    assert(s_frame_stopped && s_last_command.turn_percent == 0);
}
static void test_obstacle_gates_and_t(void)
{
    reset(); NEXT(true, 0);
    enter_state(CAR_STATE_LINE_SEARCH, now);
    tick(true, false, 0, 160, 5); tick(true, false, 0, 160, 5);
    assert(s_state != CAR_STATE_AVOID_STOP);
    reset(); NEXT(true, 0); NEXT(true, 0); NEXT(true, 0);
    tick(true, true, 0, 100, 5); tick(true, true, 0, 100, 5);
    assert(s_state == CAR_STATE_AVOID_STOP && s_obstacle_consumed);
    enter_state(CAR_STATE_RUN, now); s_finish_armed = true;
    for (int i = 0; i < 10; i++) NEXT(true, 0);
    assert(ball_starts == 0 && s_state == CAR_STATE_RUN); /* no 1s handoff */
    tick(true, true, 0, 100, 5); tick(true, true, 0, 100, 5);
    assert(s_state == CAR_STATE_RUN);
    frame.wide_black_rows = 4;
    NEXT(true, 0); assert(s_last_command.forward_percent == 0);
    NEXT(true, 0); assert(s_state == CAR_STATE_T_HANDOFF);
    for (int i = 0; i < 4; i++) NEXT(true, 0);
    assert(s_state == CAR_STATE_BALL_TASK && ball_starts == 1);
}
static void publish_motion(int a, int b, int c)
{
    motion.sequence++; motion.sample_ms = now;
    motion.interval_ms = 200;
    motion.pulses[0] = a; motion.pulses[1] = b; motion.pulses[2] = c;
}
static void test_strafe_feedback_and_hold(void)
{
    reset(); NEXT(true, 0);
    enter_state(CAR_STATE_AVOID_STRAFE_LEFT, now);
    NEXT(true, 0);
    assert(s_last_command.strafe_percent == 18);
    now += 500; publish_motion(10, 35, 6);
    tick(true, true, 0, 0, ULTRASONIC_INVALID);
    assert(s_strafe_c_trim > 0 && s_strafe_b_trim < 0);
    assert(pwm[0] < 0 && pwm[1] < 0 && pwm[2] < 0);
    for (int i = 0; i < 3; i++) {
        now += 200; publish_motion(10, 25, 0);
        tick(true, true, 0, 0, ULTRASONIC_INVALID);
    }
    assert(s_state == CAR_STATE_AVOID_HOLD);
    assert(pwm[0] == 0 && pwm[1] == 0 && pwm[2] == 0);
    for (int i = 0; i < 20; i++) NEXT(false, 0);
    assert(s_state == CAR_STATE_AVOID_HOLD);
    reset(); NEXT(true, 0); enter_state(CAR_STATE_AVOID_FIND_LINE, now);
    s_state_enter_ms = now - AVOID_FIND_LINE_MS;
    NEXT(false, 0);
    assert(s_state == CAR_STATE_AVOID_HOLD);
    reset(); NEXT(true, 0); enter_state(CAR_STATE_AVOID_STRAFE_LEFT, now);
    NEXT(true, 0);
    for (int i = 0; i < 8; i++) NEXT(true, 0);
    assert(s_state == CAR_STATE_AVOID_HOLD); /* no encoder feed cannot pass silently */
    reset(); NEXT(true, 0); enter_state(CAR_STATE_AVOID_STRAFE_LEFT, now);
    NEXT(true, 0);
    tick(false, true, 0, 701, ULTRASONIC_INVALID);
    NEXT(true, 0);
    assert(s_state == CAR_STATE_AVOID_HOLD); /* camera outage cannot eat travel timer */
}
int main(void)
{
    test_turn_lock(); test_route_sequence(); test_camera_fallback_and_timeout();
    test_raw_frames_and_filter(); test_obstacle_gates_and_t();
    test_strafe_feedback_and_hold();
    puts("PASS: direction lock, new-frame exit, fallback, stale-frame stop, obstacle gates, T handoff, strafe stall/balance");
    return 0;
}
