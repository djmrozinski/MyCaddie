idf_component_register(
    SRCS "ble_server.c"
    INCLUDE_DIRS "."
    REQUIRES bt nimble motor_driver
)
