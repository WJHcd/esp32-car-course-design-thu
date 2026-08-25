#include "infrared.h"
#include "pins.h"
#include "driver/gpio.h"

/* ============================================================
 * 四路红外循迹模块 LQ-R4CHV
 *   - 黑色 PCB，4 路红外对管，输出 OUT1~OUT4（数字量）
 *   - 极性（2026-08-25 实测修正）：黑线=低电平、白底=高电平
 *     （常规 LM393 极性，见 pins.h 的 IR_LINE_LEVEL）
 *
 * a/b/c/d 映射（2026-08-25 按装车物理位置修正）：
 *     a = OUT4,  b = OUT3,  c = OUT2,  d = OUT1
 *     即 a→b→c→d 对应 OUT4→OUT3→OUT2→OUT1 的物理顺序
 *     （a/b 为一侧，c/d 为另一侧，与误差公式 E_V=(a*1.9+b*1.2)-(c*1.2+d*1.9) 一致）。
 * ⚠️ 若实际装车后左右顺序相反，调换 s_ir_out 表或 infrared_read()
 *    里的 a/b/c/d 赋值即可，并在 test_infrared 里观察核对。
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

    pat->d = val[0];   /* OUT1 → d */
    pat->c = val[1];   /* OUT2 → c */
    pat->b = val[2];   /* OUT3 → b */
    pat->a = val[3];   /* OUT4 → a */
}

bool infrared_all_black(const infrared_pattern_t *pat)
{
    return (pat->a == 0 && pat->b == 0 && pat->c == 0 && pat->d == 0);
}
