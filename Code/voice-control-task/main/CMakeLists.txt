# 音频输入默认走 UAC（摄像头 USB 三合一，与视频同线）。
# 若改用独立 I2S 数字麦：把 audio_in_uac.c 换回 audio_in_i2s.c，
# 并把下方 usb/esp_timer 依赖换回 esp_driver_i2s（audio_in_i2s.c 引脚见其顶部）。
idf_component_register(SRCS "main.c" "audio_in_uac.c" "voice_cmd.c"
                            "app_menu.c" "bt_remote.c" "ble_control.c"
                            "expression.c" "voice_remote.c" "motion.c"
                            "tft_disp.c" "status_disp.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           esp_psram        # Required for CONFIG_SPIRAM
                           esp_driver_gpio
                           esp_driver_ledc # 电机 PWM（D24A，20kHz）
                           esp_driver_spi  # TFT 状态屏（SPI2）
                           usb              # USB Host（UAC 麦克风，esp32s3）
                           esp_timer       # BLE 控制环/协议租约时间戳
                           nvs_flash       # BLE 协议栈参数存储
                           bt              # NimBLE 主机（CONFIG_BT_NIMBLE_ENABLED）
                           espressif__esp-sr  # 离线语音识别（ESP-SR，组件管理器安装，目录名即组件名）
                       )
