/* Host-only regression: cc -std=c11 -I main tests/test_green_detect.c
 * main/ball_detect.c -lm -o test_green_detect
 * Synthetic RGB tests are NOT real camera calibration. */
#include "ball_detect.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t rgb[240 * 160 * 3];
static ball_field_result_t field;

static void rect(int x, int y, int w, int h, int r, int g, int b)
{
    for (int j = y; j < y + h; j++) {
        for (int i = x; i < x + w; i++) {
            uint8_t *p = rgb + (j * 240 + i) * 3;
            p[0] = r; p[1] = g; p[2] = b;
        }
    }
}
static void reset(void)
{
    ball_detect_init(240, 160);
    memset(rgb, 220, sizeof(rgb));
}
static void frames(int n)
{
    while (n--) ball_detect_process_rgb888(rgb, &field);
}
static void patch(int r, int g, int b, int expected)
{
    reset(); rect(100, 40, 8, 8, r, g, b); frames(3);
    assert(field.marker_count == expected);
}
int main(void)
{
    patch(120, 120, 120, 0);  /* neutral silver */
    patch(120, 126, 120, 0);  /* weak green cast */
    patch(220, 230, 220, 0);  /* bright gray with green cast */
    patch(210, 240, 60, 1);   /* fluorescent yellow-green */
    patch(225, 231, 217, 1); /* pale green mixed with white */
    patch(239, 245, 226, 1); /* bright pale yellow-green */
    patch(100, 140, 100, 1); /* saturated ordinary green */
    patch(240, 240, 80, 0);  /* yellow, not green */
    reset(); rect(100, 40, 1, 1, 100, 140, 100); frames(4);
    assert(field.marker_count == 0);
    reset(); rect(100, 40, 16, 4, 210, 240, 60); frames(3);
    assert(field.marker_count == 1); /* perspective-flattened marker */
    reset(); rect(100, 120, 16, 16, 210, 240, 60); frames(4);
    assert(field.marker_count == 0); /* chassis bottom 30% */
    reset(); rect(100, 40, 4, 4, 210, 240, 60); frames(2);
    assert(field.marker_count == 0);
    frames(1); assert(field.marker_count == 1);
    memset(rgb, 220, sizeof(rgb)); frames(1);
    assert(field.marker_count == 0);
    rect(100, 40, 4, 4, 210, 240, 60); frames(1);
    assert(field.marker_count == 0); /* no stale confirmation */
    frames(2); assert(field.marker_count == 1);
    reset();
    rect(16, 16, 16, 16, 210, 240, 60);
    rect(56, 16, 16, 16, 210, 240, 60); frames(20);
    rect(100, 40, 24, 24, 210, 240, 60); frames(3);
    assert(field.marker_count == 2);
    assert(field.marker[0].x_ratio > 0.45f && field.marker[0].x_ratio < 0.48f);
    reset();
    rect(20, 40, 16, 16, 200, 50, 50);
    rect(160, 40, 16, 16, 40, 60, 200); frames(1);
    assert(field.red_ball.detected && field.blue_ball.detected);
    puts("green detection regressions: PASS");
    return 0;
}
