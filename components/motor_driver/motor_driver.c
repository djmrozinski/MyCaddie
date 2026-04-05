/*
 * motor_driver.c - Phase 1: LEDC PWM motor control
 * TODO: Set GPIO pin numbers to match your hardware wiring.
 */
#include "motor_driver.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdlib.h>

static const char *TAG = "motor_driver";

#define MOTOR_L_PWM_GPIO    18
#define MOTOR_L_DIR_GPIO    19
#define MOTOR_R_PWM_GPIO    20
#define MOTOR_R_DIR_GPIO    21

#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_FREQ_HZ        5000
#define LEDC_RESOLUTION     LEDC_TIMER_10_BIT
#define LEDC_CH_LEFT        LEDC_CHANNEL_0
#define LEDC_CH_RIGHT       LEDC_CHANNEL_1
#define DUTY_MAX            1023
#define DIR_CHANGE_RAMP_MAX_PCT  50   /* ramp to this % or below before toggling direction */
#define DIR_CHANGE_RAMP_STEP_PCT  5   /* duty step size per ramp tick                      */
#define DIR_CHANGE_RAMP_DELAY_MS  10  /* delay between ramp steps (ms)                     */

/* Per-motor runtime state for direction-change ramp protection */
static motor_dir_t s_current_dir[2]  = { MOTOR_FORWARD, MOTOR_FORWARD };
static uint8_t     s_current_pct[2]  = { 0, 0 };

static void set_duty_raw(ledc_channel_t ch, uint8_t pct)
{
    uint32_t duty = (pct * DUTY_MAX) / 100;
    ledc_set_duty(LEDC_MODE, ch, duty);
    ledc_update_duty(LEDC_MODE, ch);
}

void motor_driver_init(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_RESOLUTION,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t ch_left = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CH_LEFT,
        .timer_sel  = LEDC_TIMER, .gpio_num = MOTOR_L_PWM_GPIO,
        .duty = 0, .hpoint = 0,
    };
    ledc_channel_config_t ch_right = {
        .speed_mode = LEDC_MODE, .channel = LEDC_CH_RIGHT,
        .timer_sel  = LEDC_TIMER, .gpio_num = MOTOR_R_PWM_GPIO,
        .duty = 0, .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_left));
    ESP_ERROR_CHECK(ledc_channel_config(&ch_right));

    gpio_config_t dir_cfg = {
        .pin_bit_mask = (1ULL << MOTOR_L_DIR_GPIO) | (1ULL << MOTOR_R_DIR_GPIO),
        .mode         = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&dir_cfg));
    motor_stop_all();
    ESP_LOGI(TAG, "Motor driver initialised");
}

void motor_set(motor_side_t side, motor_dir_t dir, uint8_t speed_pct)
{
    uint8_t max = CONFIG_GCC_MAX_SPEED_PCT;
    if (speed_pct > max) speed_pct = max;

    ledc_channel_t ch = (side == MOTOR_LEFT) ? LEDC_CH_LEFT : LEDC_CH_RIGHT;
    int dir_gpio      = (side == MOTOR_LEFT) ? MOTOR_L_DIR_GPIO : MOTOR_R_DIR_GPIO;

    /* Direction-change protection: ramp down to DIR_CHANGE_RAMP_MAX_PCT before
     * toggling the direction GPIO — required to avoid destroying the RioRand
     * power transistors under hard commutation reversal at speed. */
    if (dir != s_current_dir[side] && s_current_pct[side] > DIR_CHANGE_RAMP_MAX_PCT) {
        ESP_LOGD(TAG, "Dir change motor %d: ramping %d%% → %d%%",
                 side, s_current_pct[side], DIR_CHANGE_RAMP_MAX_PCT);
        int ramp = (int)s_current_pct[side];
        while (ramp > DIR_CHANGE_RAMP_MAX_PCT) {
            ramp -= DIR_CHANGE_RAMP_STEP_PCT;
            if (ramp < DIR_CHANGE_RAMP_MAX_PCT) ramp = DIR_CHANGE_RAMP_MAX_PCT;
            set_duty_raw(ch, (uint8_t)ramp);
            vTaskDelay(pdMS_TO_TICKS(DIR_CHANGE_RAMP_DELAY_MS));
        }
        s_current_pct[side] = (uint8_t)ramp;
    }

    /* Now safe to toggle direction and apply target speed */
    gpio_set_level(dir_gpio, (dir == MOTOR_FORWARD) ? 0 : 1);
    s_current_dir[side] = dir;

    set_duty_raw(ch, speed_pct);
    s_current_pct[side] = speed_pct;
}

void motor_stop_all(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH_LEFT,  0);
    ledc_set_duty(LEDC_MODE, LEDC_CH_RIGHT, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CH_LEFT);
    ledc_update_duty(LEDC_MODE, LEDC_CH_RIGHT);
    s_current_pct[MOTOR_LEFT]  = 0;
    s_current_pct[MOTOR_RIGHT] = 0;
}

void motor_drive(int8_t throttle, int8_t steering)
{
    int left  = throttle + steering;
    int right = throttle - steering;
    if (left  >  100) left  =  100;
    if (left  < -100) left  = -100;
    if (right >  100) right =  100;
    if (right < -100) right = -100;
    motor_set(MOTOR_LEFT,  left  >= 0 ? MOTOR_FORWARD : MOTOR_REVERSE, (uint8_t)abs(left));
    motor_set(MOTOR_RIGHT, right >= 0 ? MOTOR_FORWARD : MOTOR_REVERSE, (uint8_t)abs(right));
}

/* ---------------------------------------------------------------------------
 * Phase 2 — motor control task + BLE watchdog
 * ---------------------------------------------------------------------------
 * The queue holds a single entry. motor_ctrl_post() uses xQueueOverwrite()
 * so a newer BLE command always displaces a stale unread one. The task blocks
 * on the queue with a timeout equal to GCC_BLE_WATCHDOG_MS; if no command
 * arrives in that window it calls motor_stop_all() as a fail-safe.
 * --------------------------------------------------------------------------- */

typedef struct { int8_t throttle; int8_t steering; } motor_cmd_t;

static QueueHandle_t s_cmd_queue = NULL;

#define MOTOR_CTRL_TASK_PRIORITY  5
#define MOTOR_CTRL_TASK_STACK     2048
#define MOTOR_CTRL_TASK_CORE      1

static void motor_ctrl_task(void *arg)
{
    motor_cmd_t cmd;
    while (1) {
        if (xQueueReceive(s_cmd_queue, &cmd,
                          pdMS_TO_TICKS(CONFIG_GCC_BLE_WATCHDOG_MS)) == pdTRUE) {
            motor_drive(cmd.throttle, cmd.steering);
        } else {
            /* Watchdog fired — no command within the timeout window */
            ESP_LOGW(TAG, "BLE watchdog: no command for %dms, stopping motors",
                     CONFIG_GCC_BLE_WATCHDOG_MS);
            motor_stop_all();
        }
    }
}

void motor_ctrl_start(void)
{
    s_cmd_queue = xQueueCreate(1, sizeof(motor_cmd_t));
    configASSERT(s_cmd_queue);
    xTaskCreatePinnedToCore(motor_ctrl_task, "motor_ctrl",
                            MOTOR_CTRL_TASK_STACK, NULL,
                            MOTOR_CTRL_TASK_PRIORITY, NULL,
                            MOTOR_CTRL_TASK_CORE);
    ESP_LOGI(TAG, "Motor control task started (watchdog %dms)",
             CONFIG_GCC_BLE_WATCHDOG_MS);
}

bool motor_ctrl_post(int8_t throttle, int8_t steering)
{
    if (!s_cmd_queue) return false;
    motor_cmd_t cmd = { .throttle = throttle, .steering = steering };
    return xQueueOverwrite(s_cmd_queue, &cmd) == pdTRUE;
}
