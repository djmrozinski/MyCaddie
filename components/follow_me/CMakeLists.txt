idf_component_register(
    SRCS "follow_me.c"
    INCLUDE_DIRS "."
    REQUIRES motor_driver imu_driver freertos
)
