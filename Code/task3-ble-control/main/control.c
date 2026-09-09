#include "control.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static void stop(control_t *c)
{
    c->f = c->t = c->s = 0;
    c->button_hold = false;
    c->active_button = 0;
}
void control_link(control_t *c, bool connected)
{
    *c = (control_t){.connected = connected};
}

const char *control_command(control_t *c, const char *data, size_t len, int64_t now)
{
    if (len && data[0] == '!') {
        /* Adafruit Control Pad: ! B ASCII-button ASCII-pressed checksum.
         * Sum of all five bytes modulo 256 must equal 0xff. */
        uint8_t sum = 0;
        for (size_t i = 0; i < len; ++i) sum += (uint8_t)data[i];
        if (len != 5 || data[1] != 'B' || data[2] < '1' || data[2] > '8' ||
            (data[3] != '0' && data[3] != '1') || sum != 0xff) {
            stop(c); return "ERR pad packet";
        }
        unsigned button = data[2] - '0';
        if (data[3] == '0') {
            /* A delayed release from an older button must not cancel a new one. */
            if (c->button_hold && c->active_button == button) stop(c);
            return "RELEASE";
        }
        if (button == 2) { stop(c); c->armed = false; return "ESTOP"; }
        if (!c->connected) { stop(c); return "ERR disconnected"; }
        if (button == 1) { stop(c); c->armed = true; return "ARMED"; }
        if (!c->armed) { stop(c); return "ERR send ARM"; }
        /* Single-button control: latest press wins. Duplicate press does not
         * extend the hard hold limit. Earlier held keys never resume implicitly. */
        if (c->button_hold && c->active_button == button) return "HELD";
        stop(c);
        switch (button) {
        case 3: c->s = STRAFE_PCT; break;
        case 4: c->s = -STRAFE_PCT; break;
        case 5: c->f = DRIVE_PCT; break;
        case 6: c->f = -DRIVE_PCT; break;
        case 7: c->t = -TURN_PCT; break;
        case 8: c->t = TURN_PCT; break;
        }
        c->button_hold = true; c->active_button = button; c->last_motion_ms = now;
        return "HOLD";
    }
    char text[32];
    if (!len || len >= sizeof(text)) { stop(c); return "ERR length"; }
    for (size_t i = 0; i < len; ++i) {
        if ((unsigned char)data[i] < 32 && data[i] != '\r' && data[i] != '\n') {
            stop(c); return "ERR format";
        }
    }
    memcpy(text, data, len);
    while (len && (text[len-1] == '\r' || text[len-1] == '\n' || text[len-1] == ' ')) --len;
    text[len] = 0;
    if (!strcmp(text, "X") || !strcmp(text, "ESTOP")) {
        stop(c); c->armed = false; return "ESTOP";
    }
    if (!strcmp(text, "S") || !strcmp(text, "STOP")) { stop(c); return "STOP"; }
    if (!c->connected) { stop(c); return "ERR disconnected"; }
    if (!strcmp(text, "ARM")) { stop(c); c->armed = true; return "ARMED"; }
    if (!strcmp(text, "?")) return c->armed ? "ARMED" : "DISARMED";
    if (!c->armed) { stop(c); return "ERR send ARM"; }
    int f = 0, t = 0, s = 0;
    if (!strcmp(text, "F")) f = DRIVE_PCT;
    else if (!strcmp(text, "B")) f = -DRIVE_PCT;
    else if (!strcmp(text, "L")) t = -TURN_PCT;
    else if (!strcmp(text, "R")) t = TURN_PCT;
    else if (!strcmp(text, "Q")) s = STRAFE_PCT;
    else if (!strcmp(text, "E")) s = -STRAFE_PCT;
    else if (!strncmp(text, "M,", 2)) {
        char *p = text + 2, *end;
        int *axes[] = {&f, &t, &s};
        for (int i = 0; i < 3; ++i) {
            if (!(*p == '-' || *p == '+' || (*p >= '0' && *p <= '9'))) goto invalid;
            long v = strtol(p, &end, 10);
            if (end == p || v < -AXIS_MAX_PCT || v > AXIS_MAX_PCT) goto invalid;
            if ((i < 2 && *end != ',') || (i == 2 && *end != 0)) goto invalid;
            *axes[i] = (int)v;
            p = end + (i < 2);
        }
    } else goto invalid;
    stop(c);
    c->f = f; c->t = t; c->s = s; c->last_motion_ms = now;
    return "OK";
invalid:
    stop(c);
    return "ERR command/range";
}

bool control_tick(control_t *c, int64_t now)
{
    bool moving = c->f || c->t || c->s;
    int64_t limit = c->button_hold ? BUTTON_HOLD_MAX_MS : COMMAND_LEASE_MS;
    if (!c->connected || !c->armed || now - c->last_motion_ms >= limit) {
        stop(c); return moving;
    }
    return false;
}

void control_mix(int f, int t, int s, float out[3])
{
    /* A, B, C: calibrated same as task2 walking.c, strafe positive = left. */
    out[0] = -f + t - s;
    out[1] = -t * TURN_B_GAIN - s * STRAFE_B_GAIN;
    out[2] = (f + t - s) * MOTOR_C_GAIN;
    float peak = WHEEL_MAX_PCT;
    for (int i = 0; i < 3; ++i) if (fabsf(out[i]) > peak) peak = fabsf(out[i]);
    for (int i = 0; i < 3; ++i) out[i] *= WHEEL_MAX_PCT / peak;
}
