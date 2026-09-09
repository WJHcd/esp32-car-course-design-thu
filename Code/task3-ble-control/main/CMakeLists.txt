idf_component_register(SRCS "main.c" "control.c" "motor.c"
    INCLUDE_DIRS "."
    REQUIRES bt nvs_flash esp_timer esp_driver_gpio esp_driver_ledc)
