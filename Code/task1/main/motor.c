#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

/* ========== 引脚分配（ESP32-S3 ↔ D24A 驱动板） ==========
 * 避开：GPIO0(BOOT)、3(strapping)、19/20(USB)、26~37(flash/PSRAM)、
 *       38(WS2812)、39~42(USB-JTAG)、43/44(UART0 console)、45/46(strapping)
 * -------------------------------------------------------------
 * STBY    -> GPIO1   (全局使能，高有效)
 * PWMA/B/C -> GPIO2 / GPIO6 / GPIO9   (LEDC, 20kHz)
 * AIN1/2  -> GPIO4 / GPIO5
 * BIN1/2  -> GPIO7 / GPIO8
 * CIN1/2  -> GPIO10 / GPIO11
 * 编码器 E1A/B E2A/B E3A/B -> GPIO12~17 (见 encoder.c)
 * ========================================================== */

#define GPIO_STBY   GPIO_NUM_1

#define GPIO_PWMA   GPIO_NUM_2
#define GPIO_PWMB   GPIO_NUM_6
#define GPIO_PWMC   GPIO_NUM_9

#define GPIO_AIN1   GPIO_NUM_4
#define GPIO_AIN2   GPIO_NUM_5
#define GPIO_BIN1   GPIO_NUM_7
#define GPIO_BIN2   GPIO_NUM_8
#define GPIO_CIN1   GPIO_NUM_10
#define GPIO_CIN2   GPIO_NUM_11

#define MOTOR_PWM_FREQ_HZ   20000
#define MOTOR_PWM_RES       LEDC_TIMER_10_BIT   /* 0~1023 */
#define MOTOR_PWM_MAX       1023

static const gpio_num_t s_dir_pins[MOTOR_NUM][2] = {
    { GPIO_AIN1, GPIO_AIN2 },
    { GPIO_BIN1, GPIO_BIN2 },
    { GPIO_CIN1, GPIO_CIN2 },
};

static const gpio_num_t s_pwm_gpio[MOTOR_NUM] = { GPIO_PWMA, GPIO_PWMB, GPIO_PWMC };
static const ledc_channel_t s_pwm_ch[MOTOR_NUM] = { LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2 };

void motor_init(void)
{
    /* STBY：高电平使能 */
    gpio_config_t stby_cfg = {
        .pin_bit_mask = 1ULL << GPIO_STBY,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&stby_cfg);
    gpio_set_level(GPIO_STBY, 1);

    /* 6 个方向引脚 */
    uint64_t dir_mask = 0;
    for (int i = 0; i < MOTOR_NUM; i++) {
        dir_mask |= (1ULL << s_dir_pins[i][0]) | (1ULL << s_dir_pins[i][1]);
    }
    gpio_config_t dir_cfg = {
        .pin_bit_mask = dir_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&dir_cfg);

    /* LEDC 定时器：20kHz, 10bit */
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = MOTOR_PWM_RES,
        .freq_hz = MOTOR_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    /* 3 路 PWM 通道 */
    for (int i = 0; i < MOTOR_NUM; i++) {
        ledc_channel_config_t ch_cfg = {
            .gpio_num = s_pwm_gpio[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = s_pwm_ch[i],
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
        motor_set_dir(i, MOTOR_DIR_COAST);
    }
}

void motor_set_dir(int id, motor_dir_t dir)
{
    if (id < 0 || id >= MOTOR_NUM) return;

    bool in1, in2;
    switch (dir) {
    case MOTOR_DIR_FORWARD:  in1 = 1; in2 = 0; break;
    case MOTOR_DIR_BACKWARD: in1 = 0; in2 = 1; break;
    case MOTOR_DIR_BRAKE:    in1 = 0; in2 = 0; break;
    case MOTOR_DIR_COAST:
    default:                 in1 = 1; in2 = 1; break;
    }
    gpio_set_level(s_dir_pins[id][0], in1);
    gpio_set_level(s_dir_pins[id][1], in2);
}

void motor_set_speed(int id, int percent)
{
    if (id < 0 || id >= MOTOR_NUM) return;
    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    motor_dir_t dir = (percent >= 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    uint32_t duty = (uint32_t)((percent < 0 ? -percent : percent) * MOTOR_PWM_MAX / 100);

    motor_set_dir(id, dir);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id]));
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_set_speed(i, 0);
        motor_set_dir(i, MOTOR_DIR_BRAKE);
    }
}
