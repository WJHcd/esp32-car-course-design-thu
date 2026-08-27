#include "test.h"
#include "infrared.h"
#include "pins.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* ============================================================
 * 红外检测测试：串口打印 4 路原始电平 + a/b/c/d + 全黑判断
 * 用法：把车放到白纸/跑道上，用手遮挡某一对红外对管或放黑线，
 *       观察对应位变化；同时核对 a/b/c/d 与物理左右是否一致。
 * ============================================================ */
void test_infrared(void)
{
    infrared_init();
    printf("\n========== 红外循迹检测测试 ==========\n");
    printf("GPIO: OUT1=%d OUT2=%d OUT3=%d OUT4=%d\n",
           IR_GPIO_OUT1, IR_GPIO_OUT2, IR_GPIO_OUT3, IR_GPIO_OUT4);
    printf("实测黑线电平=IR_LINE_LEVEL(%d)：算法中 1=黑线、0=白底\n", IR_LINE_LEVEL);
    printf("物理顺序：a(左外)=OUT4, b=OUT3, c=OUT2, d(右外)=OUT1\n\n");

    while (1) {
        infrared_pattern_t pat;
        uint8_t levels = infrared_gpio_levels();
        uint8_t lines = infrared_line_mask();
        infrared_read(&pat);

        float error = 0.0f;
        bool has_line = infrared_get_error(&pat, &error);

        printf("GPIO OUT1~4=%d%d%d%d | BLACK=%d%d%d%d | a=%u b=%u c=%u d=%u | ",
               (levels >> 0) & 1, (levels >> 1) & 1,
               (levels >> 2) & 1, (levels >> 3) & 1,
               (lines >> 0) & 1, (lines >> 1) & 1,
               (lines >> 2) & 1, (lines >> 3) & 1,
               pat.a, pat.b, pat.c, pat.d);
        if (has_line) {
            printf("error=%+.2f%s\n", error,
                   infrared_all_black(&pat) ? " 全黑" : "");
        } else {
            printf("丢线(全白)\n");
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
