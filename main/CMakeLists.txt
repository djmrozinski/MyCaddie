idf_component_register(
    SRCS "app_main.c"
    INCLUDE_DIRS "."
    REQUIRES motor_driver ble_server imu_driver follow_me
)
