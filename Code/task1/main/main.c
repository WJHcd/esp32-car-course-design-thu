#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
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
 *   4 = 仅红外巡线测试（驱动电机；绝不初始化或读取超声波）
 * ============================================================ */
#define TASK1_MODE  4

/*
 * 仅红外巡线模式：用于尚未安装/标定 HC-SR04 时的全部落地循迹测试。
 * 不访问 GPIO39/40，不触发超声波，也不会进入任何避障状态。
 */
static void app_infrared_line_only(void)
{
    infrared_init();
    walking_init();
    walking_set_output_enabled(true);

    printf("\n========== 仅红外巡线测试模式 ==========\n");
    printf("超声波未初始化、未读取；避障已禁用。控制周期 20ms。\n");
    printf("当前为低速/直线防抖/锐角出口确认标定，请先低速测试。\n\n");

    uint32_t tick = 0;
    car_state_t last_state = walking_get_state();
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        infrared_pattern_t pat;
        infrared_read(&pat);

        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        walking_update(&pat, ULTRASONIC_INVALID, now_ms);

        /*
         * 丢线/找线/锐角恢复期间逐拍打印，状态切换瞬间也打印；
         * 平稳巡线时仍 25 拍一次。否则两次打印间隔的 25 个控制周期不可见，
         * 方向记忆何时翻转无从定位。
         */
        const car_state_t state = walking_get_state();
        const bool state_changed = (state != last_state);
        last_state = state;

        if (state_changed ||
            state == CAR_STATE_LINE_SEARCH ||
            state == CAR_STATE_CORNER_SEARCH ||
            tick % 25 == 0) {
            const walking_command_t cmd = walking_get_last_command();
            printf("[%6lu ms] IR-only state=%-16s a=%u b=%u c=%u d=%u "
                   "err=%+.2f cmd(F=%+d,T=%+d)\n",
                   (unsigned long)now_ms,
                   walking_state_name(state),
                   pat.a, pat.b, pat.c, pat.d,
                   walking_get_last_error(),
                   cmd.forward_percent, cmd.turn_percent);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        tick++;
    }
}

/* 综合模式：红外 + 超声波 + 行走策略，20ms 一个控制周期 */
void app_integrated(void)
{
    infrared_init();
    ultrasonic_init();
    walking_init();
    walking_set_output_enabled(true);

    printf("\n========== 综合模式：循迹 + 超声波避障 ==========\n");
    printf("目标控制周期 20ms，超声波每 100ms 单次测量（最长阻塞约 30ms）\n\n");

    uint32_t tick = 0;
    uint16_t last_distance = ULTRASONIC_INVALID;
    car_state_t last_state = walking_get_state();
    TickType_t last_wake = xTaskGetTickCount();

    while (1) {
        infrared_pattern_t pat;
        infrared_read(&pat);

        /* 只有测量发生的这一拍才把距离交给状态机，避免重复消费旧样本。 */
        uint16_t distance_sample = ULTRASONIC_INVALID;
        if (tick % 5 == 0) {
            distance_sample = ultrasonic_measure_cm();
            last_distance = distance_sample;
        }

        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        walking_update(&pat, distance_sample, now_ms);

        /*
         * 丢线/找线/锐角恢复期间逐拍打印，状态切换瞬间也打印；
         * 平稳巡线时仍 25 拍（500ms）一次。
         */
        const car_state_t state = walking_get_state();
        const bool state_changed = (state != last_state);
        last_state = state;

        if (state_changed ||
            state == CAR_STATE_LINE_SEARCH ||
            state == CAR_STATE_CORNER_SEARCH ||
            tick % 25 == 0) {
            walking_command_t cmd = walking_get_last_command();
            if (last_distance == ULTRASONIC_INVALID) {
                printf("[%6lu ms] state=%-16s a=%u b=%u c=%u d=%u "
                       "err=%+.2f cmd(F=%+d,T=%+d) 距离=无效\n",
                       (unsigned long)now_ms,
                       walking_state_name(state),
                       pat.a, pat.b, pat.c, pat.d,
                       walking_get_last_error(),
                       cmd.forward_percent, cmd.turn_percent);
            } else {
                printf("[%6lu ms] state=%-16s a=%u b=%u c=%u d=%u "
                       "err=%+.2f cmd(F=%+d,T=%+d) 距离=%3u cm\n",
                       (unsigned long)now_ms,
                       walking_state_name(state),
                       pat.a, pat.b, pat.c, pat.d,
                       walking_get_last_error(),
                       cmd.forward_percent, cmd.turn_percent,
                       last_distance);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
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
#elif TASK1_MODE == 4
    app_infrared_line_only();
#else
    app_integrated();
#endif
}
