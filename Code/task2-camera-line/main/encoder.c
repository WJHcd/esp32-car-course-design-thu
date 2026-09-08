#include "encoder.h"
#include "pins.h"
#include "driver/pulse_cnt.h"
#include "esp_check.h"
#include "esp_timer.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/*
 * 每个轮胎转一整圈所对应的“编码器 A 相上升沿”数。
 * 这个值必须实测标定：让单个轮子精确转 10 圈，记录脉冲 N，然后填 N/10。
 * 默认 20.0 仅让程序可运行，未标定前屏幕 RPM 不能作为绝对值使用。
 */
#define ENCODER_PPR_A              20.0f
#define ENCODER_PPR_B              20.0f
#define ENCODER_PPR_C              20.0f
#define ENCODER_SAMPLE_MIN_US    200000LL
#define ENCODER_GLITCH_FILTER_NS    2000

static const int s_encoder_gpio[MOTOR_NUM] = {
    ENC_A_GPIO, ENC_B_GPIO, ENC_C_GPIO,
};

static const float s_encoder_ppr[MOTOR_NUM] = {
    ENCODER_PPR_A, ENCODER_PPR_B, ENCODER_PPR_C,
};

static pcnt_unit_handle_t s_units[MOTOR_NUM];
static encoder_rpm_t s_snapshot;
static int32_t s_odom[MOTOR_NUM];
static int64_t s_last_sample_us;
static bool s_initialized;
static SemaphoreHandle_t s_lock;
static encoder_motion_t s_motion;

void encoder_init(void)
{
    if (s_initialized) {
        return;
    }
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock != NULL);

    for (int i = 0; i < MOTOR_NUM; i++) {
        const pcnt_unit_config_t unit_cfg = {
            .low_limit = -30000,
            .high_limit = 30000,
            .flags.accum_count = true,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_units[i]));

        const pcnt_glitch_filter_config_t filter_cfg = {
            .max_glitch_ns = ENCODER_GLITCH_FILTER_NS,
        };
        ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(s_units[i], &filter_cfg));

        const pcnt_chan_config_t channel_cfg = {
            .edge_gpio_num = s_encoder_gpio[i],
            .level_gpio_num = -1,  /* 单相测速，不使用 B 相 */
        };
        pcnt_channel_handle_t channel = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(s_units[i], &channel_cfg, &channel));
        /* 仅计 A 相上升沿，PPR 标定值也必须按“上升沿数/轮圈”填写。 */
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
            channel, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));

        ESP_ERROR_CHECK(pcnt_unit_enable(s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_units[i]));
    }

    s_snapshot = (encoder_rpm_t){0};
    memset(s_odom, 0, sizeof(s_odom));
    s_last_sample_us = esp_timer_get_time();
    s_initialized = true;
}

void encoder_update(void)
{
    if (!s_initialized) {
        return;
    }
    /* TFT 与球任务均可能调用采样，避免重复读/清 PCNT 和里程互相覆盖。 */
    if (xSemaphoreTake(s_lock, 0) != pdTRUE) return;

    const int64_t now_us = esp_timer_get_time();
    const int64_t elapsed_us = now_us - s_last_sample_us;
    if (elapsed_us < ENCODER_SAMPLE_MIN_US) {
        xSemaphoreGive(s_lock);
        return;
    }

    for (int i = 0; i < MOTOR_NUM; i++) {
        int pulse_count = 0;
        ESP_ERROR_CHECK(pcnt_unit_get_count(s_units[i], &pulse_count));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[i]));

        s_odom[i] += (pulse_count < 0) ? -pulse_count : pulse_count;
        s_motion.pulses[i] = abs(pulse_count);

        const float raw_rpm = fabsf((float)pulse_count) * 60000000.0f /
                              (s_encoder_ppr[i] * (float)elapsed_us);
        /* 显示端低通，降低低速时的脉冲量化抖动；不参与车辆控制。 */
        s_snapshot.rpm[i] = s_snapshot.valid[i]
                                ? (0.70f * s_snapshot.rpm[i] + 0.30f * raw_rpm)
                                : raw_rpm;
        s_snapshot.valid[i] = true;
    }

    s_last_sample_us = now_us;
    s_motion.interval_ms = (uint32_t)(elapsed_us / 1000);
    s_motion.sample_ms = (uint32_t)(now_us / 1000);
    s_motion.sequence++;
    xSemaphoreGive(s_lock);
}

encoder_rpm_t encoder_get_rpm(void)
{
    if (!s_initialized) return (encoder_rpm_t){0};
    xSemaphoreTake(s_lock, portMAX_DELAY);
    encoder_rpm_t result = s_snapshot;
    xSemaphoreGive(s_lock);
    return result;
}

encoder_motion_t encoder_get_motion(void)
{
    if (!s_initialized) return (encoder_motion_t){0};
    xSemaphoreTake(s_lock, portMAX_DELAY);
    encoder_motion_t result = s_motion;
    xSemaphoreGive(s_lock);
    return result;
}

void encoder_odom_reset(void)
{
    if (!s_initialized) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    memset(s_odom, 0, sizeof(s_odom));
    for (int i = 0; i < MOTOR_NUM; i++) {
        ESP_ERROR_CHECK(pcnt_unit_clear_count(s_units[i]));
    }
    s_last_sample_us = esp_timer_get_time();
    xSemaphoreGive(s_lock);
}

int32_t encoder_odom_get(int motor)
{
    if (motor < 0 || motor >= MOTOR_NUM) {
        return 0;
    }
    if (!s_initialized) return 0;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    int32_t value = s_odom[motor];
    xSemaphoreGive(s_lock);
    return value;
}
