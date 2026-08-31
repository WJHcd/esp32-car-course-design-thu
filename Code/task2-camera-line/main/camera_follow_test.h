#pragma once

/*
 * 摄像头纯巡线实车测试：只初始化摄像头、行走和电机。
 * 不初始化超声、编码器、TFT，也不运行避障/终点/找线状态机。
 */
void app_camera_follow_test(void);
