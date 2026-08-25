#include "infrared.h"
#include "pins.h"
#include "driver/gpio.h"

/* ============================================================
 * 四路红外循迹模块 LQ-R4CHV
 *   - 黑色 PCB，4 路红外对管，输出 OUT1~OUT4（数字量）
 *   - 常用 LM393 比较器："检测到黑线输出低电平"（极性见 IR_LINE_LEVEL）
 *
 * a/b/c/d 映射（沿用参考 STM32 程序 sensor1~4 → d,a,b,c）：
 *     a = OUT2,  b = OUT3,  c = OUT4,  d = OUT1
 * ⚠️ 若实际装车后左右顺序相反，把 s_ir_out 表调换顺序即可，
 *    并在 test_infrared 里观察 OUT1~4 与 a/b/c/d 的对应。
 * ============================================================ */

/* OUT 口 → GPIO（顺序：OUT1..OUT4） */
static const gpio_num_t s_ir_out[IR_SENSOR_NUM] = {
    IR_GPIO_OUT1,
    IR_GPIO_OUT2,
    IR_GPIO_OUT3,
    IR_GPIO_OUT4,
};

void infrared_init(void)
{
    uint64_t mask = 0;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        mask |= (1ULL << s_ir_out[i]);
    }

    gpio_config_t cfg = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   /* 模块自带输出，不靠内部上拉 */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
}

uint8_t infrared_raw(void)
{
    uint8_t raw = 0;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        if (gpio_get_level(s_ir_out[i]) == IR_LINE_LEVEL) {
            raw |= (1u << i);   /* bit0=OUT1 ... bit3=OUT4 */
        }
    }
    return raw;
}

void infrared_read(infrared_pattern_t *pat)
{
    uint8_t raw = infrared_raw();
    int8_t val[IR_SENSOR_NUM];

    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        val[i] = (raw & (1u << i)) ? 0 : -1;   /* 0=在线上, -1=在线外 */
    }

    pat->d = val[0];   /* OUT1 */
    pat->a = val[1];   /* OUT2 */
    pat->b = val[2];   /* OUT3 */
    pat->c = val[3];   /* OUT4 */
}

bool infrared_all_black(const infrared_pattern_t *pat)
{
    return (pat->a == 0 && pat->b == 0 && pat->c == 0 && pat->d == 0);
}
