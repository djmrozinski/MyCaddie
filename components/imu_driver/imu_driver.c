/* imu_driver.c - Phase 3 stub */
#include "imu_driver.h"
#include "esp_log.h"
static const char *TAG = "imu_driver";
void imu_driver_init(void) { ESP_LOGI(TAG, "IMU driver init - Phase 3 stub"); }
bool imu_driver_get_euler(imu_euler_t *out) {
    if (!out) return false;
    out->yaw = out->pitch = out->roll = 0.0f;
    return false;
}
