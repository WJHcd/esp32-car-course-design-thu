#include <stdio.h>
#include <inttypes.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "encoder.h"

#define TEST_SPEED  50   /* 50% 占空比，空载测试 */
#define PHASE_MS    2000 /* 每阶段 2 秒 */
#define STOP_MS     500

/* 单电机转动 + 打印编码器计数 */
static void run_one(int id, const char *name, int speed)
{
    printf("\n--- %s: motor %s @ %+d%% ---\n", name,
           id == MOTOR_A ? "A" : (id == MOTOR_B ? "B" : "C"), speed);
    motor_set_speed(id, speed);
    int steps = PHASE_MS / 200;
    for (int i = 0; i < steps; i++) {
        printf("  [%s] enc%d = %" PRId32 "\n", name, id, encoder_get_count(id));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    motor_set_speed(id, 0);
    motor_set_dir(id, MOTOR_DIR_BRAKE);
    vTaskDelay(pdMS_TO_TICKS(STOP_MS));
}

/* 三电机同转（前进/后退） */
static void run_all(const char *name, int speed)
{
    printf("\n--- %s: all motors @ %+d%% ---\n", name, speed);
    motor_set_speed(MOTOR_A, speed);
    motor_set_speed(MOTOR_B, speed);
    motor_set_speed(MOTOR_C, speed);
    int steps = PHASE_MS / 200;
    for (int i = 0; i < steps; i++) {
        printf("  [%s] enc A=%" PRId32 " B=%" PRId32 " C=%" PRId32 "\n", name,
               encoder_get_count(MOTOR_A),
               encoder_get_count(MOTOR_B),
               encoder_get_count(MOTOR_C));
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    motor_stop_all();
    vTaskDelay(pdMS_TO_TICKS(STOP_MS));
}

void app_main(void)
{
    motor_init();
    encoder_init();

    printf("\n========== Motor Drive Test (D24A, 3x TB6612FNG) ==========\n");
    printf("GPIO: STBY=1 PWM(A2,B6,C9) DIR(A4/5,B7/8,C10/11) ENC(A12/13,B14/15,C16/17)\n");
    printf("速度: %d%% | 每阶段 %d ms\n\n", TEST_SPEED, PHASE_MS);

    int cycle = 0;
    while (1) {
        printf("\n########## CYCLE %d ##########\n", ++cycle);

        /* 逐个电机正转 */
        run_one(MOTOR_A, "FW",  TEST_SPEED);
        run_one(MOTOR_B, "FW",  TEST_SPEED);
        run_one(MOTOR_C, "FW",  TEST_SPEED);

        /* 逐个电机反转（验证方向） */
        run_one(MOTOR_A, "BW", -TEST_SPEED);
        run_one(MOTOR_B, "BW", -TEST_SPEED);
        run_one(MOTOR_C, "BW", -TEST_SPEED);

        /* 全部正转（前进）→ 全部反转（后退） */
        run_all("ALL-FW",  TEST_SPEED);
        run_all("ALL-BW", -TEST_SPEED);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
