/* ble_server.c - Phase 2 stub */
#include "ble_server.h"
#include "esp_log.h"
static const char *TAG = "ble_server";
static bool s_connected = false;
void ble_server_init(void) { ESP_LOGI(TAG, "BLE server init - Phase 2 stub"); }
bool ble_server_is_connected(void) { return s_connected; }
