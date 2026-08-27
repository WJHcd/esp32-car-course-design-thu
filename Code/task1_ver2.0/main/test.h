#pragma once

/* 三个功能模块的测试入口（具体选择在 main.c 的 TASK1_MODE） */
void test_infrared(void);    /* 红外检测测试 */
void test_ultrasonic(void);  /* 超声检测测试 */
void test_walking(void);     /* 行走策略状态机自检（不驱动电机） */

/* 综合模式：三个模块集成跑车 */
void app_integrated(void);
