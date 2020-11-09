#include "motor.h"

#include <stdio.h>
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <soc/gpio_reg.h>
#include <esp_timer.h>

#define MOTOR1_IN1_PIN          18
#define MOTOR1_IN2_PIN          19
#define MOTOR1_PWM_PIN          26
#define MOTOR2_IN1_PIN          22
#define MOTOR2_IN2_PIN          23
#define MOTOR2_PWM_PIN          27
#define MOTOR1_IN1_PIN_MASK     (1 << MOTOR1_IN1_PIN)
#define MOTOR1_IN2_PIN_MASK     (1 << MOTOR1_IN2_PIN)
#define MOTOR2_IN1_PIN_MASK     (1 << MOTOR2_IN1_PIN)
#define MOTOR2_IN2_PIN_MASK     (1 << MOTOR2_IN2_PIN)
#define MOTOR_GPIO_PIN_MASK     (MOTOR1_IN1_PIN_MASK | MOTOR1_IN2_PIN_MASK | MOTOR2_IN1_PIN_MASK | MOTOR2_IN2_PIN_MASK)

// Required by the motor driver
#define MOTOR_BRAKING_DURATION_FOR_CHANGING_DIR_US (100 * 1000) // 100ms

#define MOTOR_DUTY_MAX ((1 << 11) - 0)
#define MOTOR_DUTY_IDLE 660

typedef struct {
    int in1_pinmask;
    int in2_pinmask;
    int pwm_channel;
    int pwm_pin;
    MotorDir last_dir;
    bool braking_for_changing_dir;
    int64_t braking_for_changing_dir_time;
} Motor;

static Motor motors[2] = {
    {
        .in1_pinmask = MOTOR1_IN1_PIN_MASK,
        .in2_pinmask = MOTOR1_IN2_PIN_MASK,
        .pwm_channel = LEDC_CHANNEL_0,
        .pwm_pin = MOTOR1_PWM_PIN,
        .last_dir = MOTOR_STOP
    },
    {
        .in1_pinmask = MOTOR2_IN1_PIN_MASK,
        .in2_pinmask = MOTOR2_IN2_PIN_MASK,
        .pwm_channel = LEDC_CHANNEL_1,
        .pwm_pin = MOTOR2_PWM_PIN,
        .last_dir = MOTOR_STOP
    },
};

void motor_init() {
    printf("Configuring motor GPIO pins\n");
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = MOTOR_GPIO_PIN_MASK,
    };
    gpio_config(&io_conf);
    WRITE_PERI_REG(GPIO_OUT_W1TC_REG, MOTOR_GPIO_PIN_MASK);

    printf("Configuring motor PWM pins\n");
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_11_BIT,
        .freq_hz = 9000,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&ledc_timer);

    for (int i = 0; i < sizeof(motors) / sizeof(motors[0]); ++i) {
        ledc_channel_config_t ledc_channel = {
            .channel = motors[i].pwm_channel,
            .duty = 0,
            .gpio_num = motors[i].pwm_pin,
            .speed_mode = LEDC_HIGH_SPEED_MODE,
            .hpoint = 0,
            .timer_sel = LEDC_TIMER_0
        };
        ledc_channel_config(&ledc_channel);
    }
}

void motor_update(int index, MotorDir dir, int throttle, int max_throttle, int dead_zone)
{
    int duty;
    if (throttle <= dead_zone) {
        duty = 0;
    } else {
        duty = (unsigned long)throttle * (MOTOR_DUTY_MAX - MOTOR_DUTY_IDLE) / max_throttle + MOTOR_DUTY_IDLE;
    }

    if (duty < 0) {
        duty = 0;
    } else if (duty > MOTOR_DUTY_MAX) {
        duty = MOTOR_DUTY_MAX;
    }

    Motor *motor = motors + index;
    bool braking_for_changing_dir = false;
    if (motor->last_dir != MOTOR_STOP && dir != motor->last_dir) {
        int64_t now = esp_timer_get_time();
        if (motor->braking_for_changing_dir) {
            if (now - motor->braking_for_changing_dir_time >= MOTOR_BRAKING_DURATION_FOR_CHANGING_DIR_US) {
                motor->braking_for_changing_dir = false;
            } else {
                braking_for_changing_dir = true;
            }
        } else {
            motor->braking_for_changing_dir = true;
            braking_for_changing_dir = true;
            motor->braking_for_changing_dir_time = now;
        }
    }

    if (!braking_for_changing_dir) {
        motor->last_dir = dir;
    }

    int actual_duty = 0;
    if (braking_for_changing_dir || dir == MOTOR_STOP) {
        //Serial.print("stop ");
        WRITE_PERI_REG(GPIO_OUT_W1TC_REG, motor->in1_pinmask | motor->in2_pinmask);
    } else {
        actual_duty = duty;
        if (dir == MOTOR_FORWARD) {
            //Serial.print("forward ");
            WRITE_PERI_REG(GPIO_OUT_W1TS_REG, motor->in1_pinmask);
            WRITE_PERI_REG(GPIO_OUT_W1TC_REG, motor->in2_pinmask);
        } else {
            //Serial.print("backward ");
            WRITE_PERI_REG(GPIO_OUT_W1TC_REG, motor->in1_pinmask);
            WRITE_PERI_REG(GPIO_OUT_W1TS_REG, motor->in2_pinmask);
        }
    }
    ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor->pwm_channel, actual_duty);
    ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor->pwm_channel);
}
