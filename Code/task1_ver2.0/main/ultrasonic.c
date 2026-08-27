#include "ultrasonic.h"
#include "pins.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

/* ============================================================
 * HC-SR04 超声波测距
 *   原理：TRIG 发 10µs 高电平 → 模块回 40kHz 脉冲 → ECHO 高电平
 *   宽度 = 往返时间；距离(cm) = 时间(µs) × 0.0343 / 2
 *   量程 2~400cm，超出/超时判为无效。
 *
 * 实现约束：
 *   1. 触发前确认 ECHO 已回到低电平，避免把上次残留脉冲当新回波；
 *   2. 每次调用只触发一次，最长阻塞约 30ms；不在函数内连续重试；
 *   3. 两次触发至少间隔 60ms，避免前一次余波污染下一次测量；
 *   4. 障碍确认与滤波放在 walking.c，由多个独立样本完成。
 * ============================================================ */

#define US_TRIG_PULSE_US       10
#define US_ECHO_IDLE_TIMEOUT_US 2000
#define US_ECHO_TIMEOUT_US     30000
#define US_MIN_INTERVAL_US     60000

static int64_t s_last_trigger_us = -US_MIN_INTERVAL_US;

void ultrasonic_init(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << US_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&trig));
    ESP_ERROR_CHECK(gpio_set_level(US_TRIG_GPIO, 0));

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << US_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   /* ECHO 空闲为低，不能上拉 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&echo));
    s_last_trigger_us = -US_MIN_INTERVAL_US;
}

/* 等待引脚达到指定电平，成功时返回该时刻；超时返回 -1。 */
static int64_t wait_echo_level(int level, int64_t timeout_us)
{
    const int64_t started = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_GPIO) != level) {
        if (esp_timer_get_time() - started >= timeout_us) {
            return -1;
        }
    }
    return esp_timer_get_time();
}

uint16_t ultrasonic_measure_cm(void)
{
    const int64_t now = esp_timer_get_time();
    if (now - s_last_trigger_us < US_MIN_INTERVAL_US) {
        return ULTRASONIC_INVALID;
    }

    /* ECHO 必须先处于空闲低电平；持续为高通常表示接线或残留回波异常。 */
    if (wait_echo_level(0, US_ECHO_IDLE_TIMEOUT_US) < 0) {
        return ULTRASONIC_INVALID;
    }

    ESP_ERROR_CHECK(gpio_set_level(US_TRIG_GPIO, 0));
    esp_rom_delay_us(2);
    ESP_ERROR_CHECK(gpio_set_level(US_TRIG_GPIO, 1));
    esp_rom_delay_us(US_TRIG_PULSE_US);
    ESP_ERROR_CHECK(gpio_set_level(US_TRIG_GPIO, 0));
    s_last_trigger_us = esp_timer_get_time();

    const int64_t rise_us = wait_echo_level(1, US_ECHO_TIMEOUT_US);
    if (rise_us < 0) {
        return ULTRASONIC_INVALID;
    }

    const int64_t fall_us = wait_echo_level(0, US_ECHO_TIMEOUT_US);
    if (fall_us < 0) {
        return ULTRASONIC_INVALID;
    }

    const int64_t width_us = fall_us - rise_us;

    /* 距离(cm) = 脉宽(us) * 343m/s / 2；整数四舍五入。 */
    const uint32_t cm = (uint32_t)((width_us * 343 + 10000) / 20000);
    return (cm >= 2 && cm <= 400) ? (uint16_t)cm : ULTRASONIC_INVALID;
}
