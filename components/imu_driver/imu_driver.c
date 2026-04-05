/*
 * imu_driver.c - Phase 3: BNO08x IMU via SHTP/SH-2 over I2C
 *
 * Enables the ARVR Stabilised Rotation Vector report (ID 0x28) at 100 Hz
 * and converts the quaternion output to Euler angles (yaw/pitch/roll).
 *
 * Protocol overview:
 *   - Every I2C transaction begins with a 4-byte SHTP header:
 *       [len_lsb, len_msb_no_continuation, channel, seq_num]
 *   - To read: fetch 4-byte header first, extract total length, re-read
 *     the full packet (header + payload) in a second transaction.
 *   - To write: prepend 4-byte header to payload and write in one transaction.
 *   - Channel 2 (CONTROL) carries SH-2 Set Feature Commands.
 *   - Channel 3 (REPORTS) carries incoming sensor reports.
 *
 * Coordinate note: yaw = rotation about Z-axis, positive = clockwise when
 * viewed from above. Range -180..+180 degrees.
 */

#include "imu_driver.h"
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "math.h"
#include <string.h>

static const char *TAG = "imu_driver";

/* ---------------------------------------------------------------------------
 * Constants
 * --------------------------------------------------------------------------- */
#define SHTP_HEADER_LEN         4
#define MAX_PACKET_LEN          256

#define SHTP_CH_COMMAND         0
#define SHTP_CH_EXECUTABLE      1
#define SHTP_CH_CONTROL         2
#define SHTP_CH_REPORTS         3

#define SH2_REPORT_ARVR_RV      0x28    /* ARVR Stabilised Rotation Vector */
#define SH2_CMD_SET_FEATURE     0xFD
#define SET_FEATURE_PAYLOAD_LEN 17      /* bytes after SHTP header */

#define REPORT_INTERVAL_US      10000   /* 10 ms = 100 Hz */

#define I2C_SPEED_HZ            400000  /* 400 kHz fast mode */
#define I2C_TIMEOUT_MS          50

#define IMU_TASK_PRIORITY       4
#define IMU_TASK_STACK          3072
#define IMU_TASK_CORE           1

/* Q-point scale factors */
#define Q14_SCALE               (1.0f / 16384.0f)  /* quaternion components */

/* ---------------------------------------------------------------------------
 * I2C and runtime state
 * --------------------------------------------------------------------------- */
static i2c_master_bus_handle_t  s_bus_handle = NULL;
static i2c_master_dev_handle_t  s_dev_handle = NULL;

static uint8_t  s_seq[6] = {0};        /* per-channel sequence counters */
static bool     s_data_valid = false;

static imu_euler_t    s_latest   = {0};
static SemaphoreHandle_t s_mutex = NULL;

/* ---------------------------------------------------------------------------
 * Low-level SHTP I2C helpers
 * --------------------------------------------------------------------------- */

/* Read a full SHTP packet into buf (including header).
 * Returns packet payload length (0 if no data / empty packet). */
static int shtp_read(uint8_t *buf, size_t buf_len)
{
    /* Step 1: read 4-byte header to find total length */
    esp_err_t err = i2c_master_receive(s_dev_handle, buf, SHTP_HEADER_LEN,
                                       I2C_TIMEOUT_MS);
    if (err != ESP_OK) return -1;

    uint16_t total_len = (uint16_t)buf[0] | ((uint16_t)(buf[1] & 0x7F) << 8);
    if (total_len == 0 || total_len <= SHTP_HEADER_LEN) return 0;
    if (total_len > buf_len) total_len = (uint16_t)buf_len;

    /* Step 2: re-read the full packet (header + payload) */
    err = i2c_master_receive(s_dev_handle, buf, total_len, I2C_TIMEOUT_MS);
    if (err != ESP_OK) return -1;

    return (int)(total_len - SHTP_HEADER_LEN);
}

/* Write a SHTP packet.  payload points to the data after the header. */
static esp_err_t shtp_write(uint8_t channel, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t total_len = SHTP_HEADER_LEN + payload_len;
    uint8_t  buf[SHTP_HEADER_LEN + SET_FEATURE_PAYLOAD_LEN + 4]; /* largest message we send */
    if (total_len > sizeof(buf)) return ESP_ERR_NO_MEM;

    buf[0] = (uint8_t)(total_len & 0xFF);
    buf[1] = (uint8_t)(total_len >> 8);
    buf[2] = channel;
    buf[3] = s_seq[channel]++;

    memcpy(buf + SHTP_HEADER_LEN, payload, payload_len);
    return i2c_master_transmit(s_dev_handle, buf, total_len, I2C_TIMEOUT_MS);
}

/* ---------------------------------------------------------------------------
 * BNO08x sensor enable
 * --------------------------------------------------------------------------- */
static esp_err_t bno08x_enable_report(uint8_t report_id, uint32_t interval_us)
{
    uint8_t cmd[SET_FEATURE_PAYLOAD_LEN] = {
        SH2_CMD_SET_FEATURE,
        report_id,
        0x00,                                       /* feature flags */
        0x00, 0x00,                                 /* change sensitivity LSB/MSB */
        (uint8_t)(interval_us & 0xFF),              /* report interval [7:0] */
        (uint8_t)((interval_us >> 8)  & 0xFF),
        (uint8_t)((interval_us >> 16) & 0xFF),
        (uint8_t)((interval_us >> 24) & 0xFF),
        0x00, 0x00, 0x00, 0x00,                     /* batch interval */
        0x00, 0x00, 0x00, 0x00,                     /* sensor-specific config */
    };
    return shtp_write(SHTP_CH_CONTROL, cmd, sizeof(cmd));
}

/* ---------------------------------------------------------------------------
 * Quaternion → Euler conversion (degrees, ZYX convention)
 * --------------------------------------------------------------------------- */
static void quat_to_euler(float qi, float qj, float qk, float qr,
                          float *yaw, float *pitch, float *roll)
{
    *yaw   = atan2f(2.0f * (qr*qk + qi*qj),
                    1.0f - 2.0f * (qj*qj + qk*qk)) * (180.0f / (float)M_PI);
    *pitch = asinf( fmaxf(-1.0f, fminf(1.0f,
                    2.0f * (qr*qj - qk*qi))))       * (180.0f / (float)M_PI);
    *roll  = atan2f(2.0f * (qr*qi + qj*qk),
                    1.0f - 2.0f * (qi*qi + qj*qj)) * (180.0f / (float)M_PI);
}

/* ---------------------------------------------------------------------------
 * Parse an incoming SHTP reports packet
 * --------------------------------------------------------------------------- */
static void parse_packet(const uint8_t *buf, int payload_len)
{
    if (payload_len < 1) return;

    uint8_t report_id = buf[SHTP_HEADER_LEN];   /* first payload byte */

    if (report_id == SH2_REPORT_ARVR_RV && payload_len >= 14) {
        /* Bytes relative to start of payload (after 4-byte SHTP header):
         *  0: report ID
         *  1: sequence
         *  2: status/delay high
         *  3: delay low
         *  4-5:  quaternion i  (Q14 signed)
         *  6-7:  quaternion j  (Q14 signed)
         *  8-9:  quaternion k  (Q14 signed)
         * 10-11: quaternion real (Q14 signed)
         * 12-13: accuracy estimate */
        const uint8_t *p = buf + SHTP_HEADER_LEN;
        int16_t raw_i    = (int16_t)((uint16_t)p[4]  | ((uint16_t)p[5]  << 8));
        int16_t raw_j    = (int16_t)((uint16_t)p[6]  | ((uint16_t)p[7]  << 8));
        int16_t raw_k    = (int16_t)((uint16_t)p[8]  | ((uint16_t)p[9]  << 8));
        int16_t raw_real = (int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));

        float qi = raw_i    * Q14_SCALE;
        float qj = raw_j    * Q14_SCALE;
        float qk = raw_k    * Q14_SCALE;
        float qr = raw_real * Q14_SCALE;

        imu_euler_t e;
        quat_to_euler(qi, qj, qk, qr, &e.yaw, &e.pitch, &e.roll);

        xSemaphoreTake(s_mutex, portMAX_DELAY);
        s_latest    = e;
        s_data_valid = true;
        xSemaphoreGive(s_mutex);

        ESP_LOGV(TAG, "yaw=%.1f pitch=%.1f roll=%.1f", e.yaw, e.pitch, e.roll);
    }
}

/* ---------------------------------------------------------------------------
 * IMU task — polls BNO08x at 100 Hz
 * --------------------------------------------------------------------------- */
static void imu_task(void *arg)
{
    static uint8_t packet_buf[MAX_PACKET_LEN];

    /* Drain any buffered packets from boot, then enable the report */
    vTaskDelay(pdMS_TO_TICKS(100));
    shtp_read(packet_buf, sizeof(packet_buf));  /* discard advertisement */

    esp_err_t err = bno08x_enable_report(SH2_REPORT_ARVR_RV, REPORT_INTERVAL_US);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ARVR report: %s", esp_err_to_name(err));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "ARVR rotation vector enabled at %lu Hz",
             (unsigned long)(1000000 / REPORT_INTERVAL_US));

    while (1) {
        int payload_len = shtp_read(packet_buf, sizeof(packet_buf));
        if (payload_len > 0) {
            parse_packet(packet_buf, payload_len);
        }
        vTaskDelay(pdMS_TO_TICKS(10));  /* 100 Hz poll */
    }
}

/* ---------------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------------- */
void imu_driver_init(void)
{
    s_mutex = xSemaphoreCreateMutex();
    configASSERT(s_mutex);

    i2c_master_bus_config_t bus_cfg = {
        .clk_source             = I2C_CLK_SRC_DEFAULT,
        .i2c_port               = I2C_NUM_0,
        .scl_io_num             = CONFIG_GCC_IMU_SCL_GPIO,
        .sda_io_num             = CONFIG_GCC_IMU_SDA_GPIO,
        .glitch_ignore_cnt      = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_cfg, &s_bus_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = CONFIG_GCC_IMU_I2C_ADDR,
        .scl_speed_hz    = I2C_SPEED_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle));

    xTaskCreatePinnedToCore(imu_task, "imu_task",
                            IMU_TASK_STACK, NULL,
                            IMU_TASK_PRIORITY, NULL,
                            IMU_TASK_CORE);

    ESP_LOGI(TAG, "IMU driver initialised (SDA=%d SCL=%d addr=0x%02X)",
             CONFIG_GCC_IMU_SDA_GPIO, CONFIG_GCC_IMU_SCL_GPIO,
             CONFIG_GCC_IMU_I2C_ADDR);
}

bool imu_driver_get_euler(imu_euler_t *out)
{
    if (!out || !s_mutex) return false;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_latest;
    bool valid = s_data_valid;
    xSemaphoreGive(s_mutex);
    return valid;
}

bool imu_driver_get_heading(float *heading_deg)
{
    imu_euler_t e;
    if (!imu_driver_get_euler(&e)) return false;
    if (heading_deg) *heading_deg = e.yaw;
    return true;
}
