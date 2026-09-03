# 任务二：摄像头巡线 + 超声波避障
# 模块划分：
#   camera_line.c —— UVC 采集 + JPEG 解码 + 黑线质心检测（替代红外巡线）
#   ultrasonic.c  —— 超声测距（沿用任务一）
#   walking.c     —— 行走策略（沿用任务一，误差来源改为摄像头）
#   motor.c       —— 底层电机驱动（沿用任务一，D24A 3×TB6612FNG）
#   encoder.c     —— 编码器测速（TFT 显示用）
#   tft_st7735.c  —— TFT 显示（转速 + 距离）
idf_component_register(
    SRCS "ascii_preview.c" "camera_follow_test.c" "main.c" "camera_line.c" "ultrasonic.c" "walking.c" "motor.c"
         "encoder.c" "tft_st7735.c" "gimbal.c" "ball_detect.c" "ball_task.c"
         "test_ultrasonic.c"
    INCLUDE_DIRS "."
    REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_pcnt esp_driver_spi
             esp_timer esp_psram usb espressif__esp_jpeg)
