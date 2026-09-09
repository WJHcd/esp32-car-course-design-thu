#include <string.h>
#include <stdlib.h>

#include "ble_control.h"

/* ============================================================
 * BLE 手机遥控协议/安全状态机（纯逻辑）
 *
 * 由 task3-ble-control 的 control.c 原样移植（djj，2026-09-03）：
 *   - 文本命令（nRF Toolbox 等）：ARM/X/S/? 与 F/B/L/R/Q/E、M,f,t,s
 *   - Bluefruit Control Pad 按键包：!B <键1-8> <0/1> <校验>（5 字节）
 *     ↑↓=5/6 前进/后退，←→=7/8 左/右转，3/4=左/右平移，1=ARM，2=ESTOP
 *   - 按键单键生效：后按的键覆盖先按的键，松开只停当前键；
 *     旧键的延迟松开不会取消新键。
 *
 * 调用约定：GATT 回调与 20ms 控制环可能并发访问，由调用方
 * （bt_remote.c）用临界区串行化。本文件无任何锁/依赖。
 * ============================================================ */

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
