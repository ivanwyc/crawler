#include "sdkconfig.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "sbus.h"
#include "motor.h"

#include <esp_task_wdt.h>
// #include <soc/rtc_wdt.h>
// #include <soc/timer_group_struct.h>
// #include <soc/timer_group_reg.h>

#define CHANNEL_MAX                 1811
#define CHANNEL_MIN                 172
#define CHANNEL_VAL_CNT             (CHANNEL_MAX - CHANNEL_MIN + 1)
#define CHANNEL_OFFSETTED_MAX       (CHANNEL_MAX - CHANNEL_MIN)
#define CHANNEL_OFFSETTED_MID       ((CHANNEL_OFFSETTED_MAX + 1) >> 1)
#define CHANNEL_OFFSETTED_QTR       (CHANNEL_OFFSETTED_MID >> 1)
#define CHANNEL_SEG_WIDTH           (CHANNEL_VAL_CNT / 3)
#define CHANEEL_PT_1                (CHANNEL_MIN)
#define CHANEEL_PT_2                (CHANNEL_MIN + CHANNEL_SEG_WIDTH)
#define CHANEEL_PT_3                (CHANNEL_MIN + CHANNEL_SEG_WIDTH * 2)
#define CHANEEL_PT_4                (CHANNEL_MAX)

#define THROTTLE_DEAD_ZONE 50

typedef enum {
    TOGGLE_LOW,
    TOGGLE_MID,
    TOGGLE_HIGH
} ToggleValue;

int offsetChannel(int channel)
{
    return channel - CHANNEL_MIN;
}

ToggleValue getToggleChannelValue(int channel)
{
    if (channel < CHANEEL_PT_2) {
        return TOGGLE_LOW;
    } else if (channel < CHANEEL_PT_3) {
        return TOGGLE_MID;
    } else {
        return TOGGLE_HIGH;
    }
}

void loop_task(void *pvParameters)
{
    TickType_t last_wakeup = xTaskGetTickCount();
    static int64_t last_valid_state_time = -1;

    while (true) {
        SBUSState *sbus_state = sbus_get_state();
        if (!sbus_state->failsafe && sbus_state->last_frame_time > last_valid_state_time) {
            last_valid_state_time = sbus_state->last_frame_time;
            uint16_t *channels = sbus_state->channels;
            printf("%010lld %d %d %d %d %d %d %d", esp_timer_get_time(), channels[0], channels[1], channels[2], channels[3], channels[4], channels[5], sbus_state->failsafe);

            MotorDir dir = (MotorDir)getToggleChannelValue(channels[4]);
            int throttle = offsetChannel(channels[0]);
            int yaw = offsetChannel(channels[1]);

            printf(" dir:%d throttle:%d yaw:%d", dir, throttle, yaw);

            MotorDir motor1Dir, motor2Dir;
            int motor1Throttle, motor2Throttle;
            int centeredYaw = yaw - CHANNEL_OFFSETTED_MID;
            int halfYaw = abs(centeredYaw) - CHANNEL_OFFSETTED_QTR;
            int adjustedthrottle = (long)throttle * abs(halfYaw) / CHANNEL_OFFSETTED_QTR;
            MotorDir adjustedDir = halfYaw < 0 ? dir : (MOTOR_REVERSE - dir);
            if (centeredYaw >= 0) {
                motor1Throttle = adjustedthrottle;
                motor1Dir = adjustedDir;
                motor2Throttle = throttle;
                motor2Dir = dir;
            } else {
                motor2Throttle = adjustedthrottle;
                motor2Dir = adjustedDir;
                motor1Throttle = throttle;
                motor1Dir = dir;
            }

            printf(" motor1Dir:%d motor1Pwm:%d motor2Dir:%d motor2Pwm:%d", motor1Dir, motor1Throttle, motor2Dir, motor2Throttle);
            printf("\n");

            motor_update(0, motor1Dir, motor1Throttle, CHANNEL_OFFSETTED_MAX, THROTTLE_DEAD_ZONE);
            motor_update(1, motor2Dir, motor2Throttle, CHANNEL_OFFSETTED_MAX, THROTTLE_DEAD_ZONE);
        }

        // This doesn't work:
        // rtc_wdt_protect_off();
        // rtc_wdt_feed();
        // rtc_wdt_protect_on();

        // This does, but apparently it disables watchdog for other tasks as well.
        // TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
        // TIMERG0.wdt_feed = 1;                       // feed dog
        // TIMERG0.wdt_wprotect = 0;                   // write protect
        // TIMERG1.wdt_wprotect = TIMG_WDT_WKEY_VALUE; // write enable
        // TIMERG1.wdt_feed = 1;                       // feed dog
        // TIMERG1.wdt_wprotect = 0;                   // write protect

        // This sleeps for one RTOS tick at minimal, which is 10ms...
        //vTaskDelayUntil(&last_wakeup, 1);

        esp_task_wdt_reset();
    }
}

void app_main(void)
{
    printf("app_main\n");

    sbus_init();
    motor_init();

    xTaskCreate(loop_task, "LOOP_TASK", 2048, NULL, 2, NULL);
    xTaskCreate(sbus_task, "SBUS_TASK", 1024, NULL, 10, NULL);
}
