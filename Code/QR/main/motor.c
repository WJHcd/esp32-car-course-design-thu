#include "motor.h"
#include "pins.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"

/* ============================================================
 * 电机底层驱动（移植自 task1，引脚见 pins.h）
 *
 * D24A 裸功率板：STBY 全局使能 + 3×PWM(LEDC) + 6×方向 GPIO
 * LEDC：20kHz, 10bit（0~1023）
 * ============================================================ */

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
static motor_dir_t s_last_motion_dir[MOTOR_NUM] = {
    MOTOR_DIR_COAST, MOTOR_DIR_COAST, MOTOR_DIR_COAST
};
static bool s_enabled;

static void set_pwm_duty(int id, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id]));
}

void motor_init(void)
{
    /* STBY 必须最先配置为低，避免初始化期间电机误动作。 */
    gpio_config_t stby_cfg = {
        .pin_bit_mask = 1ULL << GPIO_STBY,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&stby_cfg));
    ESP_ERROR_CHECK(gpio_set_level(GPIO_STBY, 0));
    s_enabled = false;

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
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));

    for (int i = 0; i < MOTOR_NUM; i++) {
        ESP_ERROR_CHECK(gpio_set_level(s_dir_pins[i][0], 0));
        ESP_ERROR_CHECK(gpio_set_level(s_dir_pins[i][1], 0));
    }

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
        s_last_motion_dir[i] = MOTOR_DIR_COAST;
    }
}

void motor_enable(bool enable)
{
    if (!enable) {
        for (int i = 0; i < MOTOR_NUM; i++) {
            set_pwm_duty(i, 0);
            motor_set_dir(i, MOTOR_DIR_COAST);
            s_last_motion_dir[i] = MOTOR_DIR_COAST;
        }
    }
    ESP_ERROR_CHECK(gpio_set_level(GPIO_STBY, enable ? 1 : 0));
    s_enabled = enable;
}

bool motor_is_enabled(void)
{
    return s_enabled;
}

void motor_set_dir(int id, motor_dir_t dir)
{
    if (id < 0 || id >= MOTOR_NUM) return;

    bool in1, in2;
    switch (dir) {
    case MOTOR_DIR_FORWARD:  in1 = 1; in2 = 0; break;
    case MOTOR_DIR_BACKWARD: in1 = 0; in2 = 1; break;
    case MOTOR_DIR_BRAKE:    in1 = 1; in2 = 1; break;
    case MOTOR_DIR_COAST:
    default:                 in1 = 0; in2 = 0; break;
    }
    ESP_ERROR_CHECK(gpio_set_level(s_dir_pins[id][0], in1));
    ESP_ERROR_CHECK(gpio_set_level(s_dir_pins[id][1], in2));
}

void motor_set_speed(int id, int percent)
{
    if (id < 0 || id >= MOTOR_NUM) return;
    if (percent > 100)  percent = 100;
    if (percent < -100) percent = -100;

    if (percent == 0) {
        set_pwm_duty(id, 0);
        motor_set_dir(id, MOTOR_DIR_COAST);
        s_last_motion_dir[id] = MOTOR_DIR_COAST;
        return;
    }

    motor_dir_t dir = (percent > 0) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    uint32_t duty = (uint32_t)((percent < 0 ? -percent : percent) * MOTOR_PWM_MAX / 100);

    /* 带载直接换向会产生很大的冲击电流；先清 PWM，滑行 200us 再换向。 */
    if ((s_last_motion_dir[id] == MOTOR_DIR_FORWARD ||
         s_last_motion_dir[id] == MOTOR_DIR_BACKWARD) &&
        s_last_motion_dir[id] != dir) {
        set_pwm_duty(id, 0);
        motor_set_dir(id, MOTOR_DIR_COAST);
        esp_rom_delay_us(200);
    }
    motor_set_dir(id, dir);
    set_pwm_duty(id, duty);
    s_last_motion_dir[id] = dir;
}

void motor_stop_all(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        set_pwm_duty(i, 0);
        motor_set_dir(i, MOTOR_DIR_BRAKE);
        s_last_motion_dir[i] = MOTOR_DIR_BRAKE;
    }
}
