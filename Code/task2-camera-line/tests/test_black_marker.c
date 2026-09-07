/* Host-only synthetic regression; this is not real-camera calibration. */
#include "ball_detect.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t rgb[240 * 160 * 3];
static ball_field_result_t field;

static void rect(int x, int y, int w, int h, int r, int g, int b)
{
    for (int yy = y; yy < y + h; yy++) {
        for (int xx = x; xx < x + w; xx++) {
            uint8_t *p = rgb + (yy * 240 + xx) * 3;
            p[0] = r; p[1] = g; p[2] = b;
        }
    }
}

static void reset(void)
{
    ball_detect_init(240, 160);
    memset(rgb, 220, sizeof(rgb));
}

static void add_red_ball(void)
{
    rect(104, 84, 20, 20, 210, 35, 35);
}

static void frames(int n)
{
    while (n--) ball_detect_process_rgb888(rgb, &field);
}

int main(void)
{
    /* 10cm 实心黑终点：在浅色地面、球后方，连续 3 帧确认。 */
    reset(); add_red_ball(); rect(100, 40, 24, 20, 35, 35, 35);
    frames(2); assert(field.marker_count == 0);
    frames(1);
    if (field.marker_count != 1) {
        ball_marker_diag_t d;
        ball_detect_get_marker_diag(&d);
        printf("target diag: red=%d px=%lu cells=%u raw=%u shape=%u "
               "surround=%u fringe=%u geometry=%u pending=%u confirmed=%u\n",
               field.red_ball.detected, (unsigned long)d.raw_marker_pixels,
               d.occupied_cells, d.raw_candidates, d.rejected_shape,
               d.rejected_surround, d.rejected_ball_fringe,
               d.rejected_body_or_geometry, d.pending_tracks,
               d.confirmed_markers);
    }
    assert(field.marker_count == 1);

    /* 无球时黑物体绝不能独自成为终点并驱动车辆。 */
    reset(); rect(100, 40, 24, 20, 35, 35, 35);
    frames(5); assert(field.marker_count == 0);

    /* 巡线、拐角、触边场外物体和过大桌面均应被过滤。 */
    reset(); add_red_ball(); rect(30, 45, 150, 4, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);
    reset(); add_red_ball();
    rect(60, 40, 48, 4, 25, 25, 25);
    rect(60, 40, 4, 40, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);
    reset(); add_red_ball(); rect(0, 30, 24, 24, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);
    reset(); add_red_ball(); rect(35, 20, 120, 64, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);

    /* 黑块周围不是浅色地面时拒绝，普通阴影也不能按黑终点处理。 */
    reset(); add_red_ball(); rect(84, 24, 56, 52, 130, 130, 130);
    rect(100, 40, 24, 20, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);
    reset(); add_red_ball(); rect(100, 40, 24, 20, 115, 115, 115);
    frames(5); assert(field.marker_count == 0);

    /* 位于球下方的黑块不符合“球在车与终点之间”。 */
    reset(); rect(104, 52, 20, 20, 210, 35, 35);
    rect(100, 88, 24, 20, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);

    /* 底部 30% 车体屏蔽仍有效，红蓝球检测链路不受改变。 */
    reset(); add_red_ball(); rect(80, 120, 24, 20, 25, 25, 25);
    frames(5); assert(field.marker_count == 0);
    reset(); rect(20, 40, 20, 20, 210, 35, 35);
    rect(160, 40, 20, 20, 35, 50, 210); frames(1);
    assert(field.red_ball.detected && field.blue_ball.detected);

    puts("black marker synthetic regressions: PASS");
    return 0;
}
