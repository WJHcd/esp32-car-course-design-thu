#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "infrared.h"
#include "ultrasonic.h"
#include "walking.h"
#include "encoder.h"
#include "tft_st7735.h"
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
 *   5 = 平移方向自检   （驱动电机；着地观察平移/前进方向，供标定）
 *   6 = TFT 面板自检    （仅屏幕，红绿蓝黑循环；不初始化任何车载模块）
 * ============================================================ */
#define TASK1_MODE  0

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
                   "err=%+.2f cmd(F=%+d,T=%+d,S=%+d)\n",
                   (unsigned long)now_ms,
                   walking_state_name(state),
                   pat.a, pat.b, pat.c, pat.d,
                   walking_get_last_error(),
                   cmd.forward_percent, cmd.turn_percent, cmd.strafe_percent);
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
    encoder_init();
    tft_st7735_init();
    tft_st7735_start_telemetry();
    walking_set_output_enabled(true);

    printf("\n========== 综合模式：循迹 + 超声波避障 ==========\n");
    printf("目标控制周期 20ms；TFT 每 250ms 显示三路编码器 RPM 和超声距离。\n\n");

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
            if (distance_sample != ULTRASONIC_INVALID) {
                tft_st7735_set_distance(distance_sample);
            }
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
                       "err=%+.2f cmd(F=%+d,T=%+d,S=%+d) 距离=无效\n",
                       (unsigned long)now_ms,
                       walking_state_name(state),
                       pat.a, pat.b, pat.c, pat.d,
                       walking_get_last_error(),
                       cmd.forward_percent, cmd.turn_percent, cmd.strafe_percent);
            } else {
                printf("[%6lu ms] state=%-16s a=%u b=%u c=%u d=%u "
                       "err=%+.2f cmd(F=%+d,T=%+d,S=%+d) 距离=%3u cm\n",
                       (unsigned long)now_ms,
                       walking_state_name(state),
                       pat.a, pat.b, pat.c, pat.d,
                       walking_get_last_error(),
                       cmd.forward_percent, cmd.turn_percent, cmd.strafe_percent,
                       last_distance);
            }
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        tick++;
    }
}

/*
 * 平移方向标定模式：轮子着地、短促脉冲，直接观察车体实际位移方向。
 * 注意：架空轮子看不出是"平移"还是"原地旋转"（两者轮子转向几乎相同），
 * 必须着地短脉冲实测。判读：
 *   - 左移脉冲车体实际向右 → 翻转 STRAFE_SIGN_L/R（B 不变）
 *   - 左移脉冲变成原地旋转 → 翻转 STRAFE_SIGN_B
 *   - 平移带明显旋转/斜走 → 微调 STRAFE_B_GAIN
 */
static void app_strafe_test(void)
{
#define STRAFE_TEST_SPEED_PCT  12
#define STRAFE_TEST_FWD_PCT    15
#define STRAFE_TEST_MS         400   /* 短脉冲，位移几厘米，安全 */
#define STRAFE_TEST_PAUSE_MS   300

    walking_init();
    walking_set_output_enabled(true);

    printf("\n========== 平移方向自检（标定用）==========\n");
    printf("请在宽敞地面/桌面【着地】运行（勿架空）；车会循环：\n");
    printf("  左移 0.4s → 停 0.3s → 右移 0.4s → 停 0.3s → 前进 0.5s → 停 0.3s\n");
    printf("判读：左移脉冲车体应【向左平移】（不旋转、不前/后）。\n");
    printf("  走反(向右)   → 翻转 walking.c 的 STRAFE_SIGN_L/R（B 不变）\n");
    printf("  原地旋转     → 翻转 STRAFE_SIGN_B\n");
    printf("  斜走/带旋转  → 微调 STRAFE_B_GAIN\n\n");

    TickType_t last_wake = xTaskGetTickCount();
    uint32_t cycle = 0;

    while (1) {
        const uint32_t t1 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 左移(+%d)\n", (unsigned long)t1, STRAFE_TEST_SPEED_PCT);
        walking_test_drive(0, 0, STRAFE_TEST_SPEED_PCT);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_MS));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        const uint32_t t2 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 右移(-%d)\n", (unsigned long)t2, STRAFE_TEST_SPEED_PCT);
        walking_test_drive(0, 0, -STRAFE_TEST_SPEED_PCT);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_MS));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        const uint32_t t3 = (uint32_t)(esp_timer_get_time() / 1000);
        printf("[%6lu ms] 前进(+%d)\n", (unsigned long)t3, STRAFE_TEST_FWD_PCT);
        walking_test_drive(STRAFE_TEST_FWD_PCT, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(500));
        walking_test_drive(0, 0, 0);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(STRAFE_TEST_PAUSE_MS));

        printf("[%6lu ms] cycle %lu 完成\n",
               (unsigned long)(esp_timer_get_time() / 1000), (unsigned long)cycle);
        cycle++;
    }
}

/* 仅 TFT 面板诊断：不初始化红外、超声、电机或编码器。 */
static void app_tft_panel_test(void)
{
    printf("\n========== TFT 独立面板测试 ==========\n");
    printf("将循环显示：红 -> 绿 -> 蓝 -> 黑，每种 1 秒。\n");
    tft_st7735_init();
    tft_st7735_panel_test();
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
#elif TASK1_MODE == 5
    app_strafe_test();
#elif TASK1_MODE == 6
    app_tft_panel_test();
#else
    app_integrated();
#endif
}
