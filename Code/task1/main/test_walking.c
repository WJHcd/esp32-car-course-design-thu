#include "test.h"
#include "walking.h"
#include "infrared.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>

/* ============================================================
 * 行走策略测试：喂入"合成传感器数据"跑一遍状态机自检。
 * 安全：walking_set_output_enabled(false) 禁止电机输出，
 *       只打印状态转移，验证逻辑是否正确，无需跑道。
 * 脚本：巡线→右偏→左偏→全黑(遇线)→全白；随后超声波 5cm 触发避障。
 * ============================================================ */

/* 合成传感器模式：0=在线上, -1=在线外 */
static const infrared_pattern_t s_script[] = {
    { .a = -1, .b = 0, .c = 0, .d = -1 },   /* 0 中间两路在线：直线 */
    { .a = -1, .b = -1, .c = 0, .d = -1 },  /* 1 右偏（左侧离线）：左修 */
    { .a = -1, .b = 0, .c = -1, .d = -1 },  /* 2 左偏（右侧离线）：右修 */
    { .a =  0, .b = 0, .c = 0, .d =  0 },   /* 3 全黑：遇线/路口 */
    { .a = -1, .b = -1, .c = -1, .d = -1 }, /* 4 全白：已脱离线 */
};

void test_walking(void)
{
    walking_init();
    walking_set_output_enabled(false);
    printf("\n========== 行走策略状态机自检 ==========\n");
    printf("电机输出已禁用，仅打印状态机转移（逻辑安全）。\n");
    printf("脚本: 直线→右偏→左偏→全黑(遇线)→全白\n\n");

    /* 第 1 步：正常巡线阶段（每种模式跑 10 拍，模拟 20ms×10） */
    for (int step = 0; step < 5; step++) {
        for (int i = 0; i < 10; i++) {
            car_state_t before = walking_get_state();
            walking_update(&s_script[step], 100);   /* 距离 100cm：无避障 */
            car_state_t after = walking_get_state();
            if (after != before) {
                printf("[步%02d/拍%02d] %-15s -> %-15s\n", step, i,
                       walking_state_name(before), walking_state_name(after));
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    /* 第 2 步：超声波避障触发（距离 5cm）*/
    printf("\n-- 超声波 5cm 触发避障 --\n");
    car_state_t before, after;
    for (int i = 0; i < 60; i++) {   /* 覆盖 AVOIDING(23拍)+ROTATE(13拍) */
        before = walking_get_state();
        walking_update(&s_script[0], 5);
        after = walking_get_state();
        if (after != before) {
            printf("[%02d] %-15s -> %-15s\n", i,
                   walking_state_name(before), walking_state_name(after));
        }
    }

    printf("\n自检完成。\n");
    walking_set_output_enabled(true);
}
