idf_component_register(SRCS "gimbal.c" "main.c" "ball_detect.c" "kick.c" "motion.c" "find_ball.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           usb
                           esp_ringbuf
                           esp_psram        # Required for CONFIG_SPIRAM
                           esp_timer
                           espressif__esp_jpeg  # 软件 JPEG 解码（ESP32-S3 无硬件 JPEG）
                           esp_driver_gpio
                       )
