#include "gimbal.h"
#include "pins.h"
#include "driver/ledc.h"
#include "esp_log.h"

/* ============================================================
 * 俯仰舵机实现：LEDC 定时器 1（50Hz，与电机 20kHz 的定时器 0 独立），
 * 通道 6（电机占用 0~2）。脉宽 0.5~2.5ms ↔ 原始舵机角 0~180°。
 * ============================================================ */

static const char *TAG = "gimbal";

#define SERVO_LEDC_TIMER    LEDC_TIMER_1
#define SERVO_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_CHANNEL  LEDC_CHANNEL_6
#define SERVO_DUTY_MAX      ((1 << 14) - 1)   /* 14bit 分辨率 @50Hz */

static int s_tilt_angle = 0;   /* 当前记录的物理俯仰角（0=水平） */

/* 原始舵机角（0~180）→ PWM */
static void servo_write_raw(int raw_deg)
{
    if (raw_deg < 0)   raw_deg = 0;
    if (raw_deg > 180) raw_deg = 180;
    const uint32_t pulse_us = GIMBAL_SERVE_MIN_US +
        (uint32_t)((GIMBAL_SERVE_MAX_US - GIMBAL_SERVE_MIN_US) * raw_deg) / 180;
    /* 14bit@50Hz：1 tick = 20000000/16384 ≈ 1221Hz... 直接按比例算 duty */
    const uint32_t duty = (uint32_t)((uint64_t)pulse_us * SERVO_DUTY_MAX / 20000);
    ESP_ERROR_CHECK(ledc_set_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL, duty));
    ESP_ERROR_CHECK(ledc_update_duty(SERVO_LEDC_MODE, SERVO_LEDC_CHANNEL));
}

void gimbal_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode      = SERVO_LEDC_MODE,
        .timer_num       = SERVO_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_14_BIT,
        .freq_hz         = GIMBAL_SERVO_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t ch_cfg = {
        .gpio_num   = SERVO_TILT_GPIO,
        .speed_mode = SERVO_LEDC_MODE,
        .channel    = SERVO_LEDC_CHANNEL,
        .timer_sel  = SERVO_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s_tilt_angle = 0;
    gimbal_set_tilt(0);   /* 回水平位 */
    ESP_LOGI(TAG, "俯仰舵机初始化完成 GPIO%d，当前 tilt=%d°",
             SERVO_TILT_GPIO, s_tilt_angle);
}

int gimbal_set_tilt(int angle_deg)
{
    if (angle_deg < GIMBAL_TILT_LIMIT_MIN) angle_deg = GIMBAL_TILT_LIMIT_MIN;
    if (angle_deg > GIMBAL_TILT_LIMIT_MAX) angle_deg = GIMBAL_TILT_LIMIT_MAX;
    s_tilt_angle = angle_deg;
    const int raw = GIMBAL_TILT_SIGN * angle_deg + GIMBAL_TILT_OFFSET;
    servo_write_raw(raw);
    return s_tilt_angle;
}

int gimbal_get_tilt(void)
{
    return s_tilt_angle;
}
