idf_component_register(SRCS "main.c" "qr_detect.c" "qr_dance.c" "display.c" "motion.c" "motor.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           usb
                           esp_ringbuf
                           esp_psram        # Required for CONFIG_SPIRAM
                           esp_timer
                           espressif__esp_jpeg  # 软件 JPEG 解码（ESP32-S3 无硬件 JPEG）
                           esp_driver_gpio
                           esp_lcd
                           esp_driver_spi
                       )
