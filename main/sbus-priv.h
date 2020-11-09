#pragma once

#include "sbus.h"

#define SBUS_UART_PORT_NUM              UART_NUM_2
#define SBUS_UART_BAUDRATE              100000
#define SBUS_UART_BUF_SIZE              1024

#define SBUS_TIME_NEEDED_PER_FRAME      3000
#define SBUS_FRAME_BEGIN_BYTE           0x0F

#define SBUS_MAX_CHANNEL                18
#define SBUS_FLAG_CHANNEL_17            (1 << 0)
#define SBUS_FLAG_CHANNEL_18            (1 << 1)
#define SBUS_DIGITAL_CHANNEL_MIN        172
#define SBUS_DIGITAL_CHANNEL_MAX        1811

#define SBUS_FLAG_SIGNAL_LOSS           (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE       (1 << 3)

#define SBUS_CHANNEL_DATA_LENGTH        sizeof(SBUSPayload)
#define SBUS_FRAME_SIZE                 (SBUS_CHANNEL_DATA_LENGTH + 2)

typedef enum {
    RX_FRAME_PENDING = 0,
    RX_FRAME_COMPLETE = (1 << 0),
    RX_FRAME_FAILSAFE = (1 << 1),
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
    RX_FRAME_DROPPED = (1 << 3)
} RXFrameState;

typedef struct {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
} __attribute__((__packed__)) SBUSPayload;

typedef struct {
    uint8_t sync_byte;
    SBUSPayload channels;
    /**
     * The end_byte is 0x00 on FrSky and some futaba RX's, on Some SBUS2 RX's the value indicates the telemetry byte that is sent after every 4th sbus frame.
     *
     * See https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101027349
     * and
     * https://github.com/cleanflight/cleanflight/issues/590#issuecomment-101706023
     */
    uint8_t end_byte;
} __attribute__ ((__packed__)) SBUSPackedFrame;

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    SBUSPackedFrame packed;
} SBUSFrame;

typedef struct {
    SBUSFrame frame;
    uint32_t start_at_us;
    uint8_t position;
    bool done;
} SBUSFrameData;

typedef int64_t TimeUS;
typedef int64_t TimeDelta;
static TimeUS last_rc_frame_time_us = 0;

static inline TimeDelta cmp_time_us(TimeUS a, TimeUS b) { return (TimeDelta)(a - b); }
