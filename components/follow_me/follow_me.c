/*
 * follow_me.c - Phase 4: BLE RSSI follow-me with Kalman filter + IMU heading hold
 *
 * Algorithm:
 *   1. Read RSSI from the active BLE connection every 100 ms (10 Hz).
 *   2. Run a 1-D Kalman filter to smooth the noisy RSSI signal.
 *   3. Convert filtered RSSI to distance using the log-distance path loss model:
 *        d = 10 ^ ((RSSI_ref - RSSI_filtered) / (10 * n))
 *      where RSSI_ref = signal at 1 m and n = path loss exponent.
 *   4. Throttle control:
 *        distance > target + deadband  → drive forward (proportional)
 *        distance < target − deadband  → hold (stop posting, watchdog feeds zero)
 *        distance < FM_MIN_DIST_CM     → emergency stop (too close / in hand)
 *        distance > FM_MAX_DIST_CM     → stop and wait (signal too weak)
 *   5. Steering: proportional heading-hold correction via IMU yaw.
 *      Reference heading captured on first valid IMU reading after start.
 *
 * Motor control: posts via motor_ctrl_post() at 10 Hz, which also feeds the
 * BLE watchdog (500 ms timeout), so no separate keep-alive is needed.
 */

#include "follow_me.h"
#include "motor_driver.h"
#include "imu_driver.h"
#include "ble_server.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

static const char *TAG = "follow_me";

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define FM_TASK_PRIORITY        3
#define FM_TASK_STACK           3072
#define FM_TASK_CORE            1
#define FM_LOOP_MS              100         /* 10 Hz */

#define FM_MIN_DIST_CM          100         /* 1 m  — stop if closer */
#define FM_MAX_DIST_CM          1500        /* 15 m — stop if further */

/* Throttle proportional gain: 1% speed per cm of distance error, capped */
#define FM_KP_THROTTLE          0.10f
#define FM_MAX_THROTTLE         40          /* % — conservative for MVP */

/* Steering proportional gain and cap */
#define FM_KP_STEER             0.25f
#define FM_MAX_STEER            25          /* % */
#define FM_HEADING_DEADBAND_DEG 3.0f        /* degrees — no correction inside */

/* Kalman filter tuning for RSSI (dBm) */
#define KALMAN_Q                1.0f        /* process noise  */
#define KALMAN_R                9.0f        /* measurement noise (std ~3 dBm) */

/* ---------------------------------------------------------------------------
 * 1-D Kalman filter
 * --------------------------------------------------------------------------- */
typedef struct {
    float x;    /* state estimate  */
    float p;    /* error covariance */
    float q;    /* process noise    */
    float r;    /* measurement noise */
} kalman_t;

static void kalman_init(kalman_t *k, float initial, float q, float r)
{
    k->x = initial;
    k->p = 1.0f;
    k->q = q;
    k->r = r;
}

static float kalman_update(kalman_t *k, float measurement)
{
    k->p += k->q;                               /* predict */
    float gain = k->p / (k->p + k->r);          /* Kalman gain */
    k->x += gain * (measurement - k->x);        /* update estimate */
    k->p *= (1.0f - gain);                      /* update covariance */
    return k->x;
}

/* ---------------------------------------------------------------------------
 * RSSI → distance conversion (log-distance path loss model)
 * --------------------------------------------------------------------------- */
static float rssi_to_distance_cm(float rssi_filtered)
{
    float n        = CONFIG_GCC_FM_PATH_LOSS_N10 / 10.0f;
    float exponent = ((float)CONFIG_GCC_FM_RSSI_REF_DBM - rssi_filtered) / (10.0f * n);
    float dist_m   = powf(10.0f, exponent);
    return dist_m * 100.0f;
}

/* ---------------------------------------------------------------------------
 * Normalise heading error to -180..+180 degrees
 * --------------------------------------------------------------------------- */
static float normalise_heading_err(float err)
{
    while (err >  180.0f) err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* ---------------------------------------------------------------------------
 * Follow-me task
 * --------------------------------------------------------------------------- */
static volatile bool s_running = false;

static void follow_me_task(void *arg)
{
    kalman_t kf;
    bool     kf_initialised  = false;
    bool     heading_locked  = false;
    float    ref_heading      = 0.0f;

    ESP_LOGI(TAG, "Follow-me task started (target %d cm, deadband %d cm)",
             CONFIG_GCC_FOLLOW_ME_DISTANCE_CM, CONFIG_GCC_FM_DEADBAND_CM);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(FM_LOOP_MS));

        /* Need an active BLE connection for RSSI */
        if (!ble_server_is_connected()) {
            heading_locked  = false;    /* re-acquire heading on reconnect */
            kf_initialised  = false;
            motor_ctrl_post(0, 0);      /* feed watchdog, hold still */
            continue;
        }

        /* Read RSSI */
        int8_t raw_rssi = 0;
        if (!ble_server_get_rssi(&raw_rssi)) {
            motor_ctrl_post(0, 0);
            continue;
        }

        /* Kalman filter */
        if (!kf_initialised) {
            kalman_init(&kf, (float)raw_rssi, KALMAN_Q, KALMAN_R);
            kf_initialised = true;
        }
        float filtered_rssi = kalman_update(&kf, (float)raw_rssi);

        /* Distance estimate */
        float dist_cm = rssi_to_distance_cm(filtered_rssi);

        /* Safety bounds */
        if (dist_cm < FM_MIN_DIST_CM || dist_cm > FM_MAX_DIST_CM) {
            ESP_LOGD(TAG, "Out of range: %.0f cm — holding", dist_cm);
            motor_ctrl_post(0, 0);
            continue;
        }

        /* Throttle: proportional to distance error, forward only */
        float target_cm  = (float)CONFIG_GCC_FOLLOW_ME_DISTANCE_CM;
        float deadband_cm = (float)CONFIG_GCC_FM_DEADBAND_CM;
        float dist_error  = dist_cm - target_cm;

        int8_t throttle = 0;
        if (dist_error > deadband_cm) {
            float t = dist_error * FM_KP_THROTTLE;
            if (t > FM_MAX_THROTTLE) t = FM_MAX_THROTTLE;
            throttle = (int8_t)t;
        }

        /* Steering: IMU heading hold */
        int8_t steering = 0;
        float  current_heading = 0.0f;
        if (imu_driver_get_heading(&current_heading)) {
            if (!heading_locked) {
                ref_heading   = current_heading;
                heading_locked = true;
                ESP_LOGI(TAG, "Reference heading locked: %.1f°", ref_heading);
            }
            float heading_err = normalise_heading_err(current_heading - ref_heading);
            if (fabsf(heading_err) > FM_HEADING_DEADBAND_DEG) {
                float s = heading_err * FM_KP_STEER;
                if (s >  FM_MAX_STEER) s =  FM_MAX_STEER;
                if (s < -FM_MAX_STEER) s = -FM_MAX_STEER;
                steering = (int8_t)s;
            }
        }

        ESP_LOGD(TAG, "rssi=%ddBm filt=%.1f dist=%.0fcm thr=%d steer=%d",
                 raw_rssi, filtered_rssi, dist_cm, throttle, steering);

        motor_ctrl_post(throttle, steering);
    }

    motor_stop_all();
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
void follow_me_start(void)
{
    if (s_running) return;
    s_running = true;
    xTaskCreatePinnedToCore(follow_me_task, "follow_me",
                            FM_TASK_STACK, NULL,
                            FM_TASK_PRIORITY, NULL,
                            FM_TASK_CORE);
}

void follow_me_stop(void)
{
    s_running = false;
    /* task will call motor_stop_all() and delete itself on next loop */
}
