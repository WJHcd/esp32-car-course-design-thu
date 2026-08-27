# 任务一：红外循迹 + 超声波避障（三个功能模块 + 综合 main）
# 模块划分：
#   infrared.c  —— 红外检测（四路循迹）
#   ultrasonic.c—— 超声检测（HC-SR04 测距）
#   walking.c   —— 行走策略（状态机 + PID + 电机驱动）
#   motor.c     —— 底层电机驱动（从 motor-drive-test 移植，D24A 3×TB6612FNG）
# 测试：test_infrared / test_ultrasonic / test_walking（在 main.c 里用 TASK1_MODE 切换）
idf_component_register(
    SRCS "main.c" "infrared.c" "ultrasonic.c" "walking.c" "motor.c"
         "encoder.c" "tft_st7735.c"
         "test_infrared.c" "test_ultrasonic.c" "test_walking.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_pcnt esp_driver_spi esp_timer)
