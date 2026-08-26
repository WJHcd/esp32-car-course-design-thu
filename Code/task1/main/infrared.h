#pragma once
#include <stdint.h>
#include <stdbool.h>

#define IR_SENSOR_NUM  4

/* 四路循迹数据按车头朝前时从左到右排列：a、b、c、d。
 *   1 = 检测到黑线
 *   0 = 白底/未检测到线
 * 实测电气极性仍由 pins.h 的 IR_LINE_LEVEL 决定。 */
typedef struct {
    uint8_t a;   /* 左外，OUT4 */
    uint8_t b;   /* 左内，OUT3 */
    uint8_t c;   /* 右内，OUT2 */
    uint8_t d;   /* 右外，OUT1 */
} infrared_pattern_t;

/**
 * @brief 初始化四路红外循迹 GPIO（输入）
 */
void infrared_init(void);

/**
 * @brief 读取 OUT1~OUT4 的实际 GPIO 电平
 * @return bit0=OUT1 ... bit3=OUT4；1=高电平，0=低电平
 */
uint8_t infrared_gpio_levels(void);

/**
 * @brief 读取归一化后的黑线掩码
 * @return bit0=OUT1 ... bit3=OUT4；1=黑线，0=白底
 */
uint8_t infrared_line_mask(void);

/**
 * @brief 读取并组装成从左到右的 a/b/c/d 模式（1=黑线，0=白底）
 */
void infrared_read(infrared_pattern_t *pat);

/**
 * @brief 判断四路是否都在线上（全黑）
 */
bool infrared_all_black(const infrared_pattern_t *pat);

/** @brief 判断四路是否都未检测到黑线（丢线） */
bool infrared_all_white(const infrared_pattern_t *pat);

/** @brief 返回检测到黑线的传感器数量（0~4） */
uint8_t infrared_line_count(const infrared_pattern_t *pat);

/**
 * @brief 计算黑线相对车体中心的位置误差，左负右正，范围约 -3~+3
 * @return true 表示至少一路检测到线；false 表示全白，此时不写 error
 */
bool infrared_get_error(const infrared_pattern_t *pat, float *error);
