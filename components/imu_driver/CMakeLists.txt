idf_component_register(
    SRCS "imu_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_driver_i2c
    PRIV_REQUIRES esp_common
)

target_link_libraries(${COMPONENT_LIB} PRIVATE m)
