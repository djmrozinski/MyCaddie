/*
 * telemetry_task.c - Phase 2: 1 Hz battery monitor + BLE telemetry
 *
 * Reads the battery voltage from a resistor divider on CONFIG_GCC_BATT_ADC_GPIO,
 * converts it to millivolts, then notifies the connected iPhone via BLE GATT.
 *
 * Voltage divider design (adjust BATT_DIVIDER_RATIO to match your resistors):
 *   36V nominal → ~3.0V at ADC pin
 *   Example: R1 = 100 kΩ (high side), R2 = 9.1 kΩ (low side)
 *   Ratio = (R1 + R2) / R2 = 109.1 / 9.1 ≈ 12.0
 *   battery_mv = adc_pin_mv × BATT_DIVIDER_RATIO
 */

#include "telemetry_task.h"
#include "ble_server.h"
#include "motor_driver.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_check.h"
#include "esp_log.h"

static const char *TAG = "telemetry";

#define TELEM_TASK_PRIORITY   2
#define TELEM_TASK_STACK      4096
#define TELEM_TASK_CORE       0
#define TELEM_INTERVAL_MS     1000

/* Adjust to match your actual voltage divider resistors */
#define BATT_DIVIDER_RATIO    12

/* GPIO → ADC1 channel mapping on ESP32-S3 (GPIO N = ADC1_CHANNEL_(N-1)) */
#define BATT_ADC_CHANNEL      (adc_channel_t)(CONFIG_GCC_BATT_ADC_GPIO - 1)

static adc_oneshot_unit_handle_t s_adc_handle  = NULL;
static adc_cali_handle_t         s_cali_handle = NULL;
static bool                      s_cali_ok     = false;

/* ---------------------------------------------------------------------------
 * ADC initialisation
 * --------------------------------------------------------------------------- */
static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,   /* 0–3.1 V input range */
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, BATT_ADC_CHANNEL, &chan_cfg));

    /* ESP32-S3 supports curve-fitting calibration */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_1,
        .chan     = BATT_ADC_CHANNEL,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t err = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle);
    if (err == ESP_OK) {
        s_cali_ok = true;
        ESP_LOGI(TAG, "ADC calibration: curve fitting");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable — voltage readings will be approximate");
    }
}

/* ---------------------------------------------------------------------------
 * Read battery voltage in millivolts
 * --------------------------------------------------------------------------- */
static uint16_t read_battery_mv(void)
{
    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, BATT_ADC_CHANNEL, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read error: %s", esp_err_to_name(err));
        return 0;
    }

    int pin_mv = 0;
    if (s_cali_ok) {
        adc_cali_raw_to_voltage(s_cali_handle, raw, &pin_mv);
    } else {
        /* Uncalibrated: linear approximation (3100 mV full-scale at 4095) */
        pin_mv = (raw * 3100) / 4095;
    }

    return (uint16_t)(pin_mv * BATT_DIVIDER_RATIO);
}

/* ---------------------------------------------------------------------------
 * Telemetry task
 * --------------------------------------------------------------------------- */
static void telemetry_task(void *arg)
{
    static bool s_low_battery_latched = false;

    adc_init();
    ESP_LOGI(TAG, "Telemetry task started (GPIO %d, threshold %d mV)",
             CONFIG_GCC_BATT_ADC_GPIO, CONFIG_GCC_LOW_BATTERY_MV);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(TELEM_INTERVAL_MS));

        uint16_t batt_mv = read_battery_mv();
        bool motors_active = ble_server_is_connected();

        /* Low battery protection */
        if (batt_mv > 0 && batt_mv < CONFIG_GCC_LOW_BATTERY_MV) {
            if (!s_low_battery_latched) {
                s_low_battery_latched = true;
                ESP_LOGW(TAG, "LOW BATTERY: %u mV (threshold %d mV) — stopping motors",
                         batt_mv, CONFIG_GCC_LOW_BATTERY_MV);
                motor_stop_all();
            }
            motors_active = false;
        }

        ble_server_notify_telemetry(batt_mv, motors_active);

        ESP_LOGD(TAG, "Battery: %u mV | BLE: %s",
                 batt_mv, ble_server_is_connected() ? "connected" : "disconnected");
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
void telemetry_task_start(void)
{
    xTaskCreatePinnedToCore(telemetry_task, "telemetry",
                            TELEM_TASK_STACK, NULL,
                            TELEM_TASK_PRIORITY, NULL,
                            TELEM_TASK_CORE);
}
