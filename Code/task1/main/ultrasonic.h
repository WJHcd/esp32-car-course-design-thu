#pragma once
#include <stdint.h>

/* 测量无效/超时标记（0xFFFF 不会与真实距离冲突） */
#define ULTRASONIC_INVALID  0xFFFF

/**
 * @brief 初始化 HC-SR04（TRIG 输出，ECHO 输入）
 */
void ultrasonic_init(void);

/**
 * @brief 单次测距（阻塞，最长约 30ms；两次调用应至少间隔 60ms）
 * @return 距离 cm（有效范围 2~400），失败/超时返回 ULTRASONIC_INVALID
 *
 * 注意：函数不会在内部重试；障碍判断应使用多个独立样本确认。
 */
uint16_t ultrasonic_measure_cm(void);
