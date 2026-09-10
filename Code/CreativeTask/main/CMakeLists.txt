idf_component_register(SRCS
                           "main.c"
                           "usb_audio.c"
                           "voice_cmd.c"
                           "app_menu.c"
                           "status_disp.c"
                           "ram_mode.c"
                           "camera_task.c"
                           "qr_detect.c"
                           "display.c"
                           "motion.c"
                           "voice_remote.c"
                           "bt_remote.c"
                           "ble_control.c"
                           "ultrasonic.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES "gif_search.bin" "gif_attack.bin"
                                   "snd_search_16k.wav" "snd_attack_16k.wav"
                                   "snd_search_48k.wav" "snd_attack_48k.wav"
                       PRIV_REQUIRES
                           usb
                           espressif__usb_host_uac   # UAC 麦克风/喇叭（usb/uac_host.h）
                           espressif__usb_host_uvc   # UVC 摄像头（usb/uvc_host.h）
                           esp_ringbuf
                           esp_psram
                           esp_timer
                           espressif__esp_jpeg
                           espressif__quirc
                           esp_driver_gpio
                           esp_driver_ledc
                           esp_driver_spi
                           esp-sr                    # 离线语音识别
                           bt                        # NimBLE（蓝牙遥控）
                           nvs_flash
                       )
