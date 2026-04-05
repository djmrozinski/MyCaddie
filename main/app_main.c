/*
 * MyCaddie - app_main.c
 * ESP32-S3 | ESP-IDF v5.x
 */
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "motor_driver.h"
#include "ble_server.h"
#include "telemetry_task.h"
#include "imu_driver.h"
#include "follow_me.h"

static const char *TAG = "app_main";

void app_main(void)
{
    ESP_LOGI(TAG, "MyCaddie starting...");

    /* NVS must be initialised before NimBLE */
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES ||
        nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

    motor_driver_init();
    motor_ctrl_start();
    ble_server_init();
    telemetry_task_start();
    imu_driver_init();
    follow_me_start();
    ESP_LOGI(TAG, "Initialisation complete.");
}
