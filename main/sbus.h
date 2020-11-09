#pragma once

#include <inttypes.h>

#define SBUS_NUM_CHANNEL 18

typedef struct {
    int64_t last_frame_time;
    bool failsafe;
    uint16_t channels[SBUS_NUM_CHANNEL];
} SBUSState;

void sbus_init();

void sbus_task(void *arg);

SBUSState *sbus_get_state();