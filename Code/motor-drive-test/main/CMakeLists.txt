idf_component_register(SRCS "main.c" "motor.c" "encoder.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_ledc esp_driver_gpio esp_driver_pcnt)