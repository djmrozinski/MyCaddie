#pragma once
#include <stdint.h>

typedef enum { MOTOR_LEFT = 0, MOTOR_RIGHT = 1 } motor_side_t;
typedef enum { MOTOR_FORWARD = 0, MOTOR_REVERSE = 1 } motor_dir_t;

void motor_driver_init(void);
void motor_set(motor_side_t side, motor_dir_t dir, uint8_t speed_pct);
void motor_stop_all(void);
void motor_drive(int8_t throttle, int8_t steering);
