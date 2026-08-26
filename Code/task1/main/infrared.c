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

uint8_t infrared_gpio_levels(void)
{
    uint8_t levels = 0;
    for (int i = 0; i < IR_SENSOR_NUM; i++) {
        if (gpio_get_level(s_ir_out[i]) != 0) {
            levels |= (1u << i);
        }
    }
    return levels;
}

uint8_t infrared_line_mask(void)
{
    const uint8_t levels = infrared_gpio_levels();
    const uint8_t valid_mask = (1u << IR_SENSOR_NUM) - 1u;

    /* 已实测黑线为低：低电平位取反后即为黑线位。保留宏以便换模块。 */
    return IR_LINE_LEVEL ? levels : ((uint8_t)~levels & valid_mask);
}

void infrared_read(infrared_pattern_t *pat)
{
    if (pat == NULL) {
        return;
    }

    const uint8_t mask = infrared_line_mask();
    pat->d = (mask >> 0) & 1u;   /* OUT1 → 右外 */
    pat->c = (mask >> 1) & 1u;   /* OUT2 → 右内 */
    pat->b = (mask >> 2) & 1u;   /* OUT3 → 左内 */
    pat->a = (mask >> 3) & 1u;   /* OUT4 → 左外 */
}

bool infrared_all_black(const infrared_pattern_t *pat)
{
    return pat != NULL && pat->a && pat->b && pat->c && pat->d;
}

bool infrared_all_white(const infrared_pattern_t *pat)
{
    return pat != NULL && !pat->a && !pat->b && !pat->c && !pat->d;
}

uint8_t infrared_line_count(const infrared_pattern_t *pat)
{
    if (pat == NULL) {
        return 0;
    }
    return pat->a + pat->b + pat->c + pat->d;
}

bool infrared_get_error(const infrared_pattern_t *pat, float *error)
{
    if (pat == NULL || error == NULL) {
        return false;
    }

    const uint8_t count = infrared_line_count(pat);
    if (count == 0) {
        return false;
    }

    /* 从左到右权重 -3、-1、+1、+3；多路命中时取质心。 */
    const int weighted = -3 * pat->a - pat->b + pat->c + 3 * pat->d;
    *error = (float)weighted / (float)count;
    return true;
}
