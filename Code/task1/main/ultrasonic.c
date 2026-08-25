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
 * ============================================================ */

#define US_TRIG_PULSE_US   10
#define US_TIMEOUT_US      30000   /* 30ms ≈ 5m 上限，防阻塞卡死 */
#define US_SOUND_CM_PER_US 0.0343f /* 声速 343m/s → 0.0343 cm/µs */

void ultrasonic_init(void)
{
    gpio_config_t trig = {
        .pin_bit_mask = 1ULL << US_TRIG_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&trig);
    gpio_set_level(US_TRIG_GPIO, 0);

    gpio_config_t echo = {
        .pin_bit_mask = 1ULL << US_ECHO_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   /* ECHO 空闲为低，不能上拉 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&echo);
}

uint16_t ultrasonic_measure_cm(void)
{
    /* 1. 发 10µs 触发脉冲 */
    gpio_set_level(US_TRIG_GPIO, 1);
    esp_rom_delay_us(US_TRIG_PULSE_US);
    gpio_set_level(US_TRIG_GPIO, 0);

    /* 2. 等待 ECHO 变高（测距开始），超时判无效 */
    int64_t t0 = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_GPIO) == 0) {
        if (esp_timer_get_time() - t0 > US_TIMEOUT_US) {
            return ULTRASONIC_INVALID;
        }
    }

    /* 3. 测量 ECHO 高电平宽度 */
    int64_t t1 = esp_timer_get_time();
    while (gpio_get_level(US_ECHO_GPIO) == 1) {
        if (esp_timer_get_time() - t1 > US_TIMEOUT_US) {
            return ULTRASONIC_INVALID;
        }
    }
    int64_t width_us = esp_timer_get_time() - t1;

    /* 4. 换算距离（往返，除以 2） */
    uint16_t cm = (uint16_t)(width_us * US_SOUND_CM_PER_US / 2.0f);

    /* 5. 有效范围过滤（HC-SR04 标称 2~400cm） */
    if (cm < 2 || cm > 400) {
        return ULTRASONIC_INVALID;
    }
    return cm;
}
