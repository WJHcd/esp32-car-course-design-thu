#include "control.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static const char *send(control_t *c, const char *s, int64_t t)
{ return control_command(c, s, strlen(s), t); }
static void pad(control_t *c, int button, bool pressed, int64_t now)
{
    char p[5] = {'!', 'B', '0'+button, pressed ? '1' : '0', 0};
    p[4] = (char)(uint8_t)~(uint8_t)(p[0]+p[1]+p[2]+p[3]);
    control_command(c, p, sizeof(p), now);
}
int main(void)
{
    control_t c = {0};
    assert(!strcmp(send(&c, "F", 0), "ERR disconnected"));
    control_link(&c, true);
    assert(!strcmp(send(&c, "F", 0), "ERR send ARM"));
    send(&c, "ARM\r\n", 0); send(&c, "F", 100);
    assert(c.f == DRIVE_PCT);
    assert(!control_tick(&c, 399));
    assert(control_tick(&c, 400) && !c.f);
    send(&c, "F", 500); send(&c, "?", 700);
    assert(control_tick(&c, 800)); /* status is not a heartbeat */
    send(&c, "F", 900); send(&c, "X", 901);
    assert(!c.armed && !c.f);
    send(&c, "F", 902); assert(!c.f);
    send(&c, "ARM", 1000); send(&c, "M,-18,15,-12", 1001);
    assert(c.f == -18 && c.t == 15 && c.s == -12);
    const char *bad[] = {"M,26,0,0", "M,18,0", "M,1,2,3junk", "M,1,2,", "M,999999999999999999,0,0", "F\nB", ""};
    for (unsigned i=0; i<sizeof(bad)/sizeof(bad[0]); ++i) {
        send(&c, "F", 1002); send(&c, bad[i], 1003);
        assert(!c.f && !c.t && !c.s);
    }
    const char nul[] = {'F',0,'B'};
    control_command(&c, nul, sizeof(nul), 1004); assert(!c.f);
    send(&c, "F", 1005); control_link(&c, false);
    assert(!c.armed && !c.f);
    control_link(&c, true); send(&c, "F", 1006); assert(!c.f);
    pad(&c, 5, true, 2000); assert(!c.f);
    pad(&c, 1, true, 2001); pad(&c, 1, false, 2002); assert(c.armed);
    pad(&c, 5, true, 2100); assert(c.f == DRIVE_PCT);
    assert(!control_tick(&c, 4100) && c.f); /* held for 2s, not a 300ms pulse */
    pad(&c, 5, false, 4200); assert(!c.f && !c.button_hold);
    pad(&c, 5, true, 5000); pad(&c, 8, true, 5100);
    pad(&c, 5, false, 5200); assert(c.t == TURN_PCT);
    pad(&c, 8, false, 5300); assert(!c.t && !c.f);
    pad(&c, 3, true, 6000); assert(c.s == STRAFE_PCT);
    pad(&c, 2, true, 6100); pad(&c, 3, false, 6200);
    assert(!c.armed && !c.s);
    pad(&c, 1, true, 7000); pad(&c, 4, true, 7100);
    pad(&c, 4, true, 9000); /* duplicate must not extend the limit */
    assert(control_tick(&c, 7100+BUTTON_HOLD_MAX_MS));
    pad(&c, 6, true, 18000); control_link(&c, false);
    assert(!c.f && !c.button_hold && !c.armed);
    control_link(&c, true); pad(&c, 1, true, 19000); pad(&c, 5, true, 19100);
    const char corrupt[] = {'!', 'B', '5', '0', 0};
    control_command(&c, corrupt, 5, 19200); assert(!c.f && !c.button_hold);
    pad(&c, 5, true, 20000); send(&c, "F", 20100);
    assert(control_tick(&c, 20400)); /* ASCII returns to the short lease */
    float out[3];
    control_mix(18,0,0,out); assert(out[0]<0 && out[1]==0 && out[2]>0);
    control_mix(0,18,0,out); assert(out[0]>0 && out[1]<0 && out[2]>0);
    control_mix(0,0,15,out); assert(out[0]<0 && out[1]<0 && out[2]<0);
    for(int f=-25;f<=25;f+=5) for(int t=-25;t<=25;t+=5) for(int s=-25;s<=25;s+=5) {
        control_mix(f,t,s,out);
        for(int i=0;i<3;++i) assert(fabsf(out[i]) <= WHEEL_MAX_PCT+0.001f);
    }
    puts("PASS: ASCII lease, Bluefruit hold/release/CRC/timeout, stop, reconnect, mixing");
}
