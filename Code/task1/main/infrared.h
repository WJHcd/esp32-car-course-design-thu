#pragma once
#include <stdint.h>
#include <stdbool.h>

#define IR_SENSOR_NUM  4

/* 四路循迹数据，语义与参考 STM32 程序保持一致：
 *   0  = 该路检测到黑线
 *   -1 = 该路在线上外（白底）
 * a/b/c/d 与 LQ-R4CHV 的 OUT 口对应关系见 infrared.c 顶部注释。 */
typedef struct {
    int8_t a;   /* OUT4 */
    int8_t b;   /* OUT3 */
    int8_t c;   /* OUT2 */
    int8_t d;   /* OUT1 */
} infrared_pattern_t;

/**
 * @brief 初始化四路红外循迹 GPIO（输入）
 */
void infrared_init(void);

/**
 * @brief 读取原始 4bit 状态（bit0=OUT1, bit1=OUT2, bit2=OUT3, bit3=OUT4）
 * @return 每个 bit：1=检测到线（按 IR_LINE_LEVEL 换算），0=在线外
 */
uint8_t infrared_raw(void);

/**
 * @brief 读取并组装成 a/b/c/d 模式（0=在线上，-1=在线外）
 */
void infrared_read(infrared_pattern_t *pat);

/**
 * @brief 判断四路是否都在线上（全黑）
 */
bool infrared_all_black(const infrared_pattern_t *pat);
