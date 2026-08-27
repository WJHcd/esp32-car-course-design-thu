#include "test.h"
#include "ultrasonic.h"
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* ============================================================
 * 超声检测测试：每 500ms 测一次距离并打印
 * 用法：把手/挡板在探头前移动，观察读数变化（2~400cm 有效）
 * ============================================================ */
void test_ultrasonic(void)
{
    ultrasonic_init();
    printf("\n========== HC-SR04 超声波检测测试 ==========\n");
    printf("GPIO: TRIG=%d ECHO=%d | 有效范围 2~400cm\n", US_TRIG_GPIO, US_ECHO_GPIO);
    printf("提示：在探头前移动手掌观察读数；显示\"无效\"检查接线\n\n");

    while (1) {
        uint16_t cm = ultrasonic_measure_cm();
        if (cm == ULTRASONIC_INVALID) {
            printf("距离: 无效（超时/超出量程）\n");
        } else {
            printf("距离: %3u cm\n", cm);
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
