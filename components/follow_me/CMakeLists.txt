idf_component_register(
    SRCS "follow_me.c"
    INCLUDE_DIRS "."
    REQUIRES motor_driver imu_driver ble_server freertos
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
