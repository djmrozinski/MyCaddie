#pragma once

/* Starts the 1 Hz telemetry task (Core 0, low priority).
 * Reads battery ADC, notifies iPhone via BLE, triggers safe-stop on low battery.
 * Call after motor_driver_init() and ble_server_init(). */
void telemetry_task_start(void);
