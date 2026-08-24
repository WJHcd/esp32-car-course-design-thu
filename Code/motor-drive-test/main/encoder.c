#include "encoder.h"
#include "motor.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

/* 编码器信号经 D24A 排针直通（板上无 MCU 处理） */
#define GPIO_E1A   GPIO_NUM_12
#define GPIO_E1B   GPIO_NUM_13
#define GPIO_E2A   GPIO_NUM_14
#define GPIO_E2B   GPIO_NUM_15
#define GPIO_E3A   GPIO_NUM_16
#define GPIO_E3B   GPIO_NUM_17

static pcnt_unit_handle_t s_units[MOTOR_NUM];

void encoder_init(void)
{
    const gpio_num_t pin_a[MOTOR_NUM] = { GPIO_E1A, GPIO_E2A, GPIO_E3A };
    const gpio_num_t pin_b[MOTOR_NUM] = { GPIO_E1B, GPIO_E2B, GPIO_E3B };

    for (int i = 0; i < MOTOR_NUM; i++) {
        pcnt_unit_config_t unit_cfg = {
            .low_limit = -32768,
            .high_limit = 32767,
        };
        ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &s_units[i]));

        pcnt_chan_config_t ch_cfg = {
            .edge_gpio_num = pin_a[i],
            .level_gpio_num = pin_b[i],
        };
        pcnt_channel_handle_t chan = NULL;
        ESP_ERROR_CHECK(pcnt_new_channel(s_units[i], &ch_cfg, &chan));

        /* A 相双边沿都计数，B 相电平反相方向 → 2 倍频 */
        ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan,
                        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
                        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
        ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan,
                        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
                        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

        ESP_ERROR_CHECK(pcnt_unit_enable(s_units[i]));
        ESP_ERROR_CHECK(pcnt_unit_start(s_units[i]));
    }
}

int32_t encoder_get_count(int id)
{
    if (id < 0 || id >= MOTOR_NUM) return 0;
    int count = 0;
    pcnt_unit_get_count(s_units[id], &count);
    return (int32_t)count;
}
