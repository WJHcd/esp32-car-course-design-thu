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
    printf("黑线电平=IR_LINE_LEVEL(%d)：在线上输出0，在线外输出-1\n", IR_LINE_LEVEL);
    printf("提示：放黑线/遮挡对管观察变化；OUT1~4 顺序与物理位置需核对\n\n");

    while (1) {
        infrared_pattern_t pat;
        uint8_t raw = infrared_raw();
        infrared_read(&pat);

        printf("OUT1~4=%d%d%d%d | a=%2d b=%2d c=%2d d=%2d | %s\n",
               (raw >> 0) & 1, (raw >> 1) & 1, (raw >> 2) & 1, (raw >> 3) & 1,
               pat.a, pat.b, pat.c, pat.d,
               infrared_all_black(&pat) ? "全黑" : "  ");

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
