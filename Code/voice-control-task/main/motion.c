#include <math.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_rom_sys.h"

#include "motion.h"

/* ============================================================
 * 电机驱动实现：D24A 三路全桥 + 三轮全向混控
 *
 * 移植自 task3-ble-control 的 motor.c（djj，2026-09-03 实车验证）：
 *   引脚（ESP32-S3 ↔ D24A）与 task2 一致：
 *     STBY -> GPIO1（高有效全局使能）
 *     PWMA/B/C -> GPIO2 / GPIO6 / GPIO9（LEDC 20kHz, 10bit）
 *     AIN1/2 -> GPIO4 / GPIO5；BIN1/2 -> GPIO7 / GPIO8
 *     CIN1/2 -> GPIO10 / GPIO11
 *   避开：GPIO0(BOOT)、3(strapping)、19/20(USB)、26~37(flash/PSRAM)、
 *         38(WS2812)、39~42(USB-JTAG)、43/44(UART0)、45/46(strapping)
 *   ⚠️ 麦克风与屏幕/编码器不共脚：task2 屏幕(13/15/17)与编码器
 *      (12/14/16) 保留时，麦克风挂 GPIO 21/41/42——见 audio_in_i2s.c。
 *
 * 驱动保护（保留原实现语义）：
 *   - <1% 指令按滑行停止（清理混控数值噪声，不做低速抬升——
 *     抬升曾致直线画龙，2026-09-01 试验结论）；
 *   - 带载换向先清 PWM、滑行 200us 再换向，避免冲击电流；
 *   - motion_enable(false) 先清 PWM + 滑行再拉低 STBY。
 * ============================================================ */

/* ========== 三轮全向混控（task2 walking.c 实车标定值） ==========
 * 分量 → 轮速：A=-f+t-s, B=-t*TURN_B - s*STRAFE_B, C=(f+t-s)*MOTOR_C
 * （strafe 正=左移；方向符号为实车标定结果，勿随意改动）
 * 多轴/单轴峰值统一归一化到轮速上限，任何指令都发得出有意义的量。 */
#define WHEEL_MAX_PCT       30.0f   /* 归一化后的轮速上限（%） */
#define TURN_B_GAIN         0.68f   /* B 轮旋转增益（标定） */
#define STRAFE_B_GAIN       1.80f   /* B 轮横移增益（标定） */
#define MOTOR_C_GAIN        0.90f   /* C 轮前向增益（标定） */

/* ========== 引脚（D24A） ========== */
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
#define MOTOR_COMMAND_ZERO_BAND_PCT   1.0f

#define MOTOR_A 0
#define MOTOR_B 1
#define MOTOR_C 2
#define MOTOR_NUM 3

typedef enum {
    MOTOR_DIR_FORWARD = 0,  /* IN1=1, IN2=0 */
    MOTOR_DIR_BACKWARD = 1, /* IN1=0, IN2=1 */
    MOTOR_DIR_BRAKE = 2,    /* IN1=1, IN2=1 (短刹) */
    MOTOR_DIR_COAST = 3,    /* IN1=0, IN2=0 (滑行) */
} motor_dir_t;

static const char *TAG = "motion";
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
static bool s_enabled = false;

static void set_pwm_duty(int id, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id], duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, s_pwm_ch[id]));
}

static void motor_set_dir(int id, motor_dir_t dir)
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

static void motor_set_speed(int id, float percent)
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
    /* 四舍五入到 duty 计数：10bit 下 1% ≈ 10.2 计数，0.5% 微调可分辨 */
    uint32_t duty = (uint32_t)(magnitude * MOTOR_PWM_MAX / 100.0f + 0.5f);

    /* 带载直接换向冲击电流大：先清 PWM，滑行 200us 再换向 */
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

void motion_init(void)
{
    /* STBY 必须先配置为低，避免初始化期间电机误动作 */
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

    ESP_LOGI(TAG, "D24A 电机就绪（STBY 低，未使能）");
}

void motion_enable(bool enable)
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

bool motion_is_enabled(void)
{
    return s_enabled;
}

void motion_drive(int fwd, int turn, int strafe)
{
    /* 未使能(STBY 低)时忽略一切指令——使能位即总开关 */
    if (!s_enabled) {
        return;
    }

    float f = (float)fwd, t = (float)turn, s = (float)strafe;

    /* 三轮全向混控（标定公式，见文件头） */
    float out[MOTOR_NUM];
    out[0] = -f + t - s;
    out[1] = -t * TURN_B_GAIN - s * STRAFE_B_GAIN;
    out[2] = (f + t - s) * MOTOR_C_GAIN;

    /* 峰值归一化到轮速上限 */
    float peak = WHEEL_MAX_PCT;
    for (int i = 0; i < MOTOR_NUM; i++) {
        if (fabsf(out[i]) > peak) {
            peak = fabsf(out[i]);
        }
    }
    for (int i = 0; i < MOTOR_NUM; i++) {
        motor_set_speed(i, out[i] * WHEEL_MAX_PCT / peak);
    }
}

void motion_stop(void)
{
    for (int i = 0; i < MOTOR_NUM; i++) {
        set_pwm_duty(i, 0);
        motor_set_dir(i, MOTOR_DIR_BRAKE);
        s_last_motion_dir[i] = MOTOR_DIR_BRAKE;
    }
}
