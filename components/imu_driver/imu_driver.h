#pragma once
#include <stdbool.h>

typedef struct {
    float yaw;    /* heading, degrees, -180..+180  */
    float pitch;  /* degrees                        */
    float roll;   /* degrees                        */
} imu_euler_t;

/* Initialise I2C, configure BNO08x ARVR rotation-vector report, start task.
 * Must be called after NVS init but before follow_me_start(). */
void imu_driver_init(void);

/* Thread-safe read of the latest Euler angles.
 * Returns true if data is valid (sensor running), false if not yet ready. */
bool imu_driver_get_euler(imu_euler_t *out);

/* Convenience: return only the yaw (heading) in degrees.
 * Returns 0.0f and false if data not yet valid. */
bool imu_driver_get_heading(float *heading_deg);
