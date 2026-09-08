#include <assert.h>
#include <stdio.h>
#include "encoder.c"
static int64_t clock_us;
int64_t esp_timer_get_time(void) { return clock_us; }
int main(void)
{
    assert(encoder_get_motion().sequence == 0);
    encoder_update(); encoder_odom_reset();
    encoder_init();
    s_units[0]->count = 10; s_units[1]->count = 20; s_units[2]->count = 8;
    clock_us = 200000;
    encoder_update();
    encoder_motion_t m = encoder_get_motion();
    assert(m.sequence == 1 && m.interval_ms == 200 && m.sample_ms == 200);
    assert(m.pulses[0] == 10 && m.pulses[1] == 20 && m.pulses[2] == 8);
    assert(encoder_odom_get(0) == 10);
    encoder_update(); assert(encoder_get_motion().sequence == 1);
    /* A second caller cannot read/clear PCNT while the first owns sampling. */
    s_units[0]->count = 5; clock_us = 400000;
    xSemaphoreTake(s_lock, 0); encoder_update();
    assert(s_units[0]->count == 5); xSemaphoreGive(s_lock);
    encoder_update(); assert(encoder_odom_get(0) == 15);
    s_units[0]->count = 7;
    encoder_odom_reset();
    assert(encoder_odom_get(0) == 0 && s_units[0]->count == 0);
    clock_us = 600000; encoder_update();
    assert(encoder_get_motion().pulses[0] == 0);
    assert(encoder_get_rpm().valid[0]);
    puts("PASS: coherent pulse samples, sampling serialization, interval, odometry reset");
    return 0;
}
