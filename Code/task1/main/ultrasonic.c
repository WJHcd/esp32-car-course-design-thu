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
 * 稳定性修复（2026-08-25 v2，撤销 v1 的 200µs 盲等——实测反而全无效）：
 *   1. 不加固定盲等：TRIG 后立即开始轮询 ECHO 上升沿；
 *   2. ECHO 电平连续 N 次采样一致才确认（消抖），抗噪声毛刺；
 *   3. 超时放宽到 40ms（400cm 往返约 23ms，留足裕量）；
 *   4. 单次测量失败时自动重试 2 次，降低间歇性丢失率。
 * ============================================================ */

#define US_TRIG_PULSE_US   10
#define US_TIMEOUT_US      40000    /* 40ms ≈ 6.8m 上限，防阻塞卡死 */
#define US_DEBOUNCE_N      3        /* ECHO 消抖：连续 N 次采样一致才确认 */
#define US_RETRY_N         2        /* 单次测量失败自动重试次数 */
#define US_SOUND_CM_PER_US 0.0343f  /* 声速 343m/s → 0.0343 cm/µs */

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

/* ECHO 电平消抖：连续 n 次采样都等于 expected 才返回 true（抗毛刺） */
static bool us_echo_stable(int expected, int n)
{
    for (int i = 0; i < n; i++) {
        if (gpio_get_level(US_ECHO_GPIO) != expected) {
            return false;
        }
    }
    return true;
}

/* 单次完整测量；失败返回 ULTRASONIC_INVALID */
static uint16_t us_measure_once(void)
{
    /* 1. 发 10µs 触发脉冲 */
    gpio_set_level(US_TRIG_GPIO, 1);
    esp_rom_delay_us(US_TRIG_PULSE_US);
    gpio_set_level(US_TRIG_GPIO, 0);

    /* 2. 等待 ECHO 变高（消抖确认），超时判无效 */
    int64_t t0 = esp_timer_get_time();
    while (!us_echo_stable(1, US_DEBOUNCE_N)) {
        if (esp_timer_get_time() - t0 > US_TIMEOUT_US) {
            return ULTRASONIC_INVALID;
        }
    }

    /* 3. 测量 ECHO 高电平宽度：持续到连续 N 次读到低为止（抗毛刺提前结束） */
    int64_t t1 = esp_timer_get_time();
    while (!us_echo_stable(0, US_DEBOUNCE_N)) {
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

uint16_t ultrasonic_measure_cm(void)
{
    /* 失败自动重试，降低间歇性丢失（HC-SR04 偶发丢回波属正常） */
    for (int i = 0; i <= US_RETRY_N; i++) {
        uint16_t cm = us_measure_once();
        if (cm != ULTRASONIC_INVALID) {
            return cm;
        }
    }
    return ULTRASONIC_INVALID;
}
