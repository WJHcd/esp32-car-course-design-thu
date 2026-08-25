#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "infrared.h"
#include "ultrasonic.h"
#include "walking.h"
#include "test.h"

/* ============================================================
 * 任务一 · 综合主程序
 *
 * 三个功能模块：
 *   infrared.c    —— 红外检测（四路循迹）
 *   ultrasonic.c  —— 超声检测（HC-SR04 测距）
 *   walking.c     —— 行走策略（状态机 + PID + 电机驱动）
 * 这里把三者综合成一个 20ms 控制循环。
 *
 * ⚠️ 运行模式切换：改下面的 TASK1_MODE 后重新编译烧录
 *   0 = 综合模式（循迹 + 超声波避障，正式跑车用）
 *   1 = 红外检测测试   （test_infrared）
 *   2 = 超声检测测试   （test_ultrasonic）
 *   3 = 行走策略自检   （test_walking，不驱动电机，安全）
 * ============================================================ */
#define TASK1_MODE  1

/* 综合模式：红外 + 超声波 + 行走策略，20ms 一个控制周期 */
void app_integrated(void)
{
    infrared_init();
    ultrasonic_init();
    walking_init();
    walking_set_output_enabled(true);

    printf("\n========== 综合模式：循迹 + 超声波避障 ==========\n");
    printf("控制周期 20ms，超声波每 200ms 测一次（阻塞最长 30ms）\n\n");

    uint32_t tick = 0;
    uint16_t distance = ULTRASONIC_INVALID;

    while (1) {
        infrared_pattern_t pat;
        infrared_read(&pat);

        /* 每 10 拍（200ms）测一次距离，避免阻塞控制循环 */
        if (tick % 10 == 0) {
            distance = ultrasonic_measure_cm();
        }

        walking_update(&pat, distance);

        /* 每 25 拍（500ms）打印一次状态 */
        if (tick % 25 == 0) {
            const char *dstr = (distance == ULTRASONIC_INVALID)
                               ? "无效" : "cm";
            if (distance == ULTRASONIC_INVALID) {
                printf("[%6lu ms] state=%-15s a=%2d b=%2d c=%2d d=%2d 距离=无效\n",
                       (unsigned long)(tick * 20),
                       walking_state_name(walking_get_state()),
                       pat.a, pat.b, pat.c, pat.d);
            } else {
                printf("[%6lu ms] state=%-15s a=%2d b=%2d c=%2d d=%2d 距离=%3u %s\n",
                       (unsigned long)(tick * 20),
                       walking_state_name(walking_get_state()),
                       pat.a, pat.b, pat.c, pat.d, distance, dstr);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
        tick++;
    }
}

void app_main(void)
{
#if   TASK1_MODE == 1
    test_infrared();
#elif TASK1_MODE == 2
    test_ultrasonic();
#elif TASK1_MODE == 3
    test_walking();
#else
    app_integrated();
#endif
}
