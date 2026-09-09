#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* PWM percentages, not RPM. Confirm directions with wheels raised first. */
#define DRIVE_PCT 18
#define TURN_PCT 18
#define STRAFE_PCT 15
#define AXIS_MAX_PCT 25
#define WHEEL_MAX_PCT 30
#define COMMAND_LEASE_MS 300
/* Bluefruit sends edges, not periodic heartbeats. Bound a lost release. */
#define BUTTON_HOLD_MAX_MS 10000
#define MOTOR_C_GAIN 0.90f
#define TURN_B_GAIN 0.68f
#define STRAFE_B_GAIN 1.80f

typedef struct {
    bool connected, armed;
    bool button_hold;
    uint8_t active_button;
    int f, t, s;
    int64_t last_motion_ms;
} control_t;

/* Caller serializes access. One ASCII command or one Bluefruit !B packet per write. */
void control_link(control_t *c, bool connected);
const char *control_command(control_t *c, const char *data, size_t len, int64_t now);
bool control_tick(control_t *c, int64_t now);
void control_mix(int f, int t, int s, float out[3]);
