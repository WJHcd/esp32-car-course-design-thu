#include "test.h"
#include "ultrasonic.h"
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* ============================================================
 * 超声检测测试：每 500ms 测一次距离并打印
 * 用法：把手/挡板在探头前移动，观察读数变化（2~400cm 有效）
 *
 * 标定模式（2026-08-27 新增）：连续采样统计，用于验证"误差<5cm"指标。
 * 用法：车固定不动，卷尺量探头到障碍物的真实距离，与屏幕/串口
 * 显示的均值对比；每 10 次测量输出一组统计（均值/最小/最大/极差）。
 * 极差(最大-最小) <5cm 且均值接近真实值即达标。
 * ============================================================ */
#define US_CALIB_WINDOW   10u

static void us_calibration_loop(void)
{
    uint16_t samples[US_CALIB_WINDOW];
    uint8_t count = 0;
    uint32_t round = 0;

    printf("\n---------- 距离标定统计（每 %u 次测量一组） ----------\n",
           (unsigned)US_CALIB_WINDOW);
    while (1) {
        const uint16_t cm = ultrasonic_measure_cm();
        if (cm == ULTRASONIC_INVALID) {
            printf("  [%2u] 无效（超时/超出量程）\n", (unsigned)(count + 1));
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        samples[count++] = cm;
        printf("  [%2u] %3u cm\n", (unsigned)count, (unsigned)cm);

        if (count >= US_CALIB_WINDOW) {
            uint16_t min = samples[0], max = samples[0];
            uint32_t sum = 0;
            for (uint8_t i = 0; i < count; i++) {
                if (samples[i] < min) min = samples[i];
                if (samples[i] > max) max = samples[i];
                sum += samples[i];
            }
            const uint16_t mean = (uint16_t)((sum + count / 2) / count);
            printf(">> 第%lu组: 均值=%ucm 最小=%u 最大=%u 极差=%ucm\n\n",
                   (unsigned long)(++round), (unsigned)mean,
                   (unsigned)min, (unsigned)max, (unsigned)(max - min));
            count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void test_ultrasonic(void)
{
    ultrasonic_init();
    printf("\n========== HC-SR04 超声波检测测试 ==========\n");
    printf("GPIO: TRIG=%d ECHO=%d | 有效范围 2~400cm\n", US_TRIG_GPIO, US_ECHO_GPIO);
    printf("提示：在探头前移动手掌观察读数；显示\"无效\"检查接线\n\n");

    us_calibration_loop();
}
