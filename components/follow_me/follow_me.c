/* follow_me.c - Phase 4 stub */
#include "follow_me.h"
#include "motor_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
static const char *TAG = "follow_me";
static bool s_running = false;
static void follow_me_task(void *arg) {
    ESP_LOGI(TAG, "Follow-me task started");
    while (s_running) { vTaskDelay(pdMS_TO_TICKS(100)); }
    motor_stop_all();
    vTaskDelete(NULL);
}
void follow_me_start(void) { s_running = true; xTaskCreatePinnedToCore(follow_me_task, "follow_me", 3072, NULL, 5, NULL, 1); }
void follow_me_stop(void)  { s_running = false; }
