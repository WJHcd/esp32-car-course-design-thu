idf_component_register(SRCS "main.c" "qr_detect.c" "qr_dance.c" "display.c" "motion.c" "motor.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES "gif_asset.bin"   # GIF 表情动画资源（tools/qr_gif_convert.py 生成）
                       PRIV_REQUIRES
                           usb
                           espressif__usb_host_uvc  # uvc_host 驱动（v2.5，usb/uvc_host.h）
                           esp_ringbuf
                           esp_psram        # Required for CONFIG_SPIRAM
                           esp_timer
                           espressif__esp_jpeg  # 软件 JPEG 解码（ESP32-S3 无硬件 JPEG）
                           espressif__quirc     # 二维码解码
                           esp_driver_gpio
                           esp_driver_ledc
                           esp_driver_spi   # TFT 用裸 spi_master（ILI9163 自绘驱动）
                       )
