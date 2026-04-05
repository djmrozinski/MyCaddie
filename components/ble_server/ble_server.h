#pragma once
#include <stdbool.h>
#include <stdint.h>

void ble_server_init(void);
bool ble_server_is_connected(void);

/* Called by telemetry_task to push a notify to the connected iPhone */
void ble_server_notify_telemetry(uint16_t battery_mv, bool motors_active);

/* Returns the current connection RSSI in dBm.
 * Returns false if not connected or RSSI unavailable. */
bool ble_server_get_rssi(int8_t *rssi_dbm);
