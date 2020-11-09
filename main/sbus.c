#include <stdatomic.h>
#include <driver/gpio.h>
#include <driver/uart.h>

#include "sbus-priv.h"

static SBUSState sbus_state_double_buffer[2];
static int sbus_state_double_buffer_index;
static SBUSState *sbus_state;

void sbus_init()
{
    uart_config_t uart_config = {
        .baud_rate = SBUS_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_2,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    ESP_ERROR_CHECK(uart_param_config(SBUS_UART_PORT_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(SBUS_UART_PORT_NUM, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(SBUS_UART_PORT_NUM, SBUS_UART_BUF_SIZE, 0, 0, NULL, 0));

    sbus_state_double_buffer_index = 0;
    sbus_state = sbus_state_double_buffer;
    sbus_state->failsafe = false;
    sbus_state->last_frame_time = esp_timer_get_time();
    for (int i = 0; i < SBUS_NUM_CHANNEL; ++i) {
        sbus_state->channels[i] = SBUS_DIGITAL_CHANNEL_MIN;
    }
}

static uint8_t sbus_decode_frame(SBUSFrameData *frame_data, SBUSState* sbus_state)
{
    SBUSPayload *channels = &frame_data->frame.packed.channels;
    uint16_t *channel_data = sbus_state->channels;

    channel_data[0] = channels->chan0;
    channel_data[1] = channels->chan1;
    channel_data[2] = channels->chan2;
    channel_data[3] = channels->chan3;
    channel_data[4] = channels->chan4;
    channel_data[5] = channels->chan5;
    channel_data[6] = channels->chan6;
    channel_data[7] = channels->chan7;
    channel_data[8] = channels->chan8;
    channel_data[9] = channels->chan9;
    channel_data[10] = channels->chan10;
    channel_data[11] = channels->chan11;
    channel_data[12] = channels->chan12;
    channel_data[13] = channels->chan13;
    channel_data[14] = channels->chan14;
    channel_data[15] = channels->chan15;

    if (channels->flags & SBUS_FLAG_CHANNEL_17) {
        channel_data[16] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        channel_data[16] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_CHANNEL_18) {
        channel_data[17] = SBUS_DIGITAL_CHANNEL_MAX;
    } else {
        channel_data[17] = SBUS_DIGITAL_CHANNEL_MIN;
    }

    if (channels->flags & SBUS_FLAG_FAILSAFE_ACTIVE) {
        // internal failsafe enabled and rx failsafe flag set
        // RX *should* still be sending valid channel data (repeated), so use it.
        return RX_FRAME_COMPLETE | RX_FRAME_FAILSAFE;
    }

    if (channels->flags & SBUS_FLAG_SIGNAL_LOSS) {
        // The received data is a repeat of the last valid data so can be considered complete.
        return RX_FRAME_COMPLETE | RX_FRAME_DROPPED;
    }

    return RX_FRAME_COMPLETE;
}

void sbus_task(void *arg)
{
    SBUSFrameData sbus_frame_data = { 0 }, *frame_data = &sbus_frame_data;

    while (1) {
        uint8_t c = 0;
        if (uart_read_bytes(SBUS_UART_PORT_NUM, &c, 1, 100 / portTICK_RATE_MS) <= 0) {
            continue;
        }

        const TimeUS nowUs = esp_timer_get_time();

        const TimeDelta sbusFrameTime = cmp_time_us(nowUs, frame_data->start_at_us);

        if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
            frame_data->position = 0;
        }

        if (frame_data->position == 0) {
            if (c != SBUS_FRAME_BEGIN_BYTE) {
                continue;
            }
            frame_data->start_at_us = nowUs;
        }

        if (frame_data->position < SBUS_FRAME_SIZE) {
            frame_data->frame.bytes[frame_data->position++] = (uint8_t)c;
            if (frame_data->position < SBUS_FRAME_SIZE) {
                frame_data->done = false;
            } else {
                last_rc_frame_time_us = nowUs;
                frame_data->done = true;
            }
        }

        if (frame_data->done) {
            frame_data->done = false;
            SBUSState *state = sbus_state_double_buffer + sbus_state_double_buffer_index;
            const uint8_t frame_status = sbus_decode_frame(frame_data, state);
            if (frame_status & RX_FRAME_COMPLETE) {
                state->last_frame_time = nowUs;
                state->failsafe = !!(frame_status & RX_FRAME_FAILSAFE);
                atomic_store(&sbus_state, state);
                sbus_state_double_buffer_index = 1 - sbus_state_double_buffer_index;
            } else {
                last_rc_frame_time_us = 0; // We received a frame but it wasn't valid new channel data
            }
        }
    }

    vTaskDelete(NULL);
}

SBUSState *sbus_get_state()
{
    return sbus_state;
}