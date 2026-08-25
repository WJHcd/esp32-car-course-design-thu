#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor.h"
#include "encoder.h"

#define TEST_SPEED             25   /* 首次架空测试从 25% 占空比开始 */
#define PHASE_MS               1000 /* 每个方向 1 秒 */
#define SAMPLE_MS              100
#define STOP_MS                700
#define MANUAL_ENCODER_TEST_MS 15000
#define START_COUNTDOWN_S      5

static const char *motor_name(int id)
{
    return id == MOTOR_A ? "A" : (id == MOTOR_B ? "B" : "C");
}

static void print_all_encoders(const char *tag)
{
    printf("  [%s] enc A=%" PRId32 " B=%" PRId32 " C=%" PRId32 "\n", tag,
           encoder_get_count(MOTOR_A),
           encoder_get_count(MOTOR_B),
           encoder_get_count(MOTOR_C));
}

static void manual_encoder_test(void)
{
    printf("\n[STEP 1] 手动编码器测试（STBY=LOW，电机不会被软件驱动）\n");
    printf("请在 %d 秒内依次手转 A/B/C 轮，观察对应 enc 是否连续变化。\n",
           MANUAL_ENCODER_TEST_MS / 1000);
    encoder_clear_all();
    for (int elapsed = 0; elapsed < MANUAL_ENCODER_TEST_MS; elapsed += 250) {
        print_all_encoders("HAND");
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}

/* 单电机、单方向测试；同时打印全部编码器以发现错线/串线。 */
static void run_one(int id, const char *direction, int speed)
{
    printf("\n--- motor %s / %s @ %+d%% ---\n",
           motor_name(id), direction, speed);
    encoder_clear_all();
    motor_set_speed(id, speed);

    int steps = PHASE_MS / SAMPLE_MS;
    for (int i = 0; i < steps; i++) {
        print_all_encoders(direction);
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_MS));
    }

    motor_stop_all();
    printf("结果 motor %s/%s: A=%" PRId32 " B=%" PRId32 " C=%" PRId32 "\n",
           motor_name(id), direction,
           encoder_get_count(MOTOR_A),
           encoder_get_count(MOTOR_B),
           encoder_get_count(MOTOR_C));
    vTaskDelay(pdMS_TO_TICKS(STOP_MS));
}

void app_main(void)
{
    motor_init();
    encoder_init();

    printf("\n========== Motor Drive Diagnostic (D24A / ESP32-S3) ==========\n");
    printf("GPIO: STBY=1 PWM(A2,B6,C9) DIR(A4/5,B7/8,C10/11) "
           "ENC(A12/13,B14/15,C16/17)\n");
    printf("速度: %d%% | 每方向 %d ms | 本程序只运行一轮\n",
           TEST_SPEED, PHASE_MS);
    printf("安全要求：轮子架空；ESP32 用 USB 时不得连接 D24A 5V；必须共地。\n");

    manual_encoder_test();

    printf("\n[STEP 2] 即将使能 D24A 并逐个测试 A/B/C。\n");
    for (int s = START_COUNTDOWN_S; s > 0; s--) {
        printf("%d...\n", s);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    motor_enable(true);

    run_one(MOTOR_A, "FW",  TEST_SPEED);
    run_one(MOTOR_A, "BW", -TEST_SPEED);
    run_one(MOTOR_B, "FW",  TEST_SPEED);
    run_one(MOTOR_B, "BW", -TEST_SPEED);
    run_one(MOTOR_C, "FW",  TEST_SPEED);
    run_one(MOTOR_C, "BW", -TEST_SPEED);

    motor_stop_all();
    vTaskDelay(pdMS_TO_TICKS(300));
    motor_enable(false);

    printf("\n========== TEST COMPLETE ==========\n");
    printf("STBY 已拉低，所有电机保持禁用。复位开发板可重新测试。\n");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
