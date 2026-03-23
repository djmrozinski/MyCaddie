/*
 * Golf Cart Controller - app_main.c
 * ESP32-S3 | ESP-IDF v5.x
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "motor_driver.h"
#include "ble_server.h"
#include "imu_driver.h"
#include "follow_me.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "Golf Cart Controller starting...");
    motor_driver_init();
    // ble_server_init();   // Uncomment for Phase 2
    // imu_driver_init();   // Uncomment for Phase 3
    // follow_me_start();   // Uncomment for Phase 4
    ESP_LOGI(TAG, "Initialisation complete.");
}
