#include "motor.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"

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

/* 混控残差清理：小于 ZERO_BAND 的指令（如 0.5% 档的数值噪声）按停止处理。
 * ⚠️ 不要在此加"低于某档就抬升到最低有效出力"的全局补偿——2026-09-01
 * 试验 9% 抬升后直线画龙：巡线的小修角本就依赖后轮 B 近似不动、仅靠
 * 前轮差速实现，抬升使 B 在 ±1~4% 小修角时以 9% 出力，横摆过冲、PD
 * 反复过调。个别阶段若出现起步嗡鸣卡死，在该阶段标定参数解决，
 * 不动驱动层（驱动层无法区分"有意的小指令"和"被死区吃掉的大意图"）。 */
#define MOTOR_COMMAND_ZERO_BAND_PCT   1.0f

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

void motor_set_speed(int id, float percent)
{
    if (id < 0 || id >= MOTOR_NUM) return;
    if (percent > 100.0f)  percent = 100.0f;
    if (percent < -100.0f) percent = -100.0f;

    float magnitude = (percent < 0.0f) ? -percent : percent;
    if (magnitude < MOTOR_COMMAND_ZERO_BAND_PCT) {
        set_pwm_duty(id, 0);
        motor_set_dir(id, MOTOR_DIR_COAST);
        s_last_motion_dir[id] = MOTOR_DIR_COAST;
        return;
    }

    motor_dir_t dir = (percent > 0.0f) ? MOTOR_DIR_FORWARD : MOTOR_DIR_BACKWARD;
    /* 四舍五入到 duty 计数：10bit 下 1% ≈ 10.2 计数，0.5% 微调可分辨。 */
    uint32_t duty = (uint32_t)(magnitude * MOTOR_PWM_MAX / 100.0f + 0.5f);

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
