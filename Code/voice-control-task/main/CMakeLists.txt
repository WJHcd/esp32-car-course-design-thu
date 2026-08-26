idf_component_register(SRCS "main.c" "audio_in_i2s.c" "voice_cmd.c" "voice_action.c" "motion.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES
                           esp_psram        # Required for CONFIG_SPIRAM
                           esp_driver_gpio
                           esp_driver_i2s  # 语音麦克风 I2S 输入
                           espressif__esp-sr  # 离线语音识别（ESP-SR，组件管理器安装，目录名即组件名）
                       )
