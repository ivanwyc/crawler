#pragma once

typedef enum {
    MOTOR_FORWARD,
    MOTOR_STOP,
    MOTOR_REVERSE,
} MotorDir;

void motor_init();

void motor_update(int index, MotorDir dir, int throttle, int max_throttle, int dead_zone);