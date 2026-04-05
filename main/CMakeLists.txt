idf_component_register(
    SRCS "telemetry_task.c" "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES motor_driver ble_server imu_driver follow_me nvs_flash esp_adc
)
