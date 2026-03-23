#pragma once
#include <stdbool.h>
typedef struct { float yaw; float pitch; float roll; } imu_euler_t;
void imu_driver_init(void);
bool imu_driver_get_euler(imu_euler_t *out);
