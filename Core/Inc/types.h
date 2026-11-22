#ifndef TYPES_H
#define TYPES_H

#include "main.h"
#include "config.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float rad_a, us_a;
    float rad_b, us_b;
    float rad_min, rad_max;
    float home_rad;
    float max_speed_rad_s;
} ServoConfig;

typedef struct {
    float current_rad;
    float target_rad;
    float start_rad;
    uint32_t start_ms;
    uint32_t duration_ms;
    uint8_t moving;
    uint16_t last_us;
} ServoState;

typedef struct {
    uint8_t cmd;
    uint8_t len;
    uint8_t payload[MAX_PAYLOAD];
} Frame;

typedef struct {
    ParserState state;
    Frame frame;
    uint8_t idx;
    uint8_t crc;
    uint32_t last_byte_ms;
} Parser;

typedef struct {
    uint8_t *buf;
    uint16_t size;
    volatile uint16_t head;
    volatile uint16_t tail;
    volatile uint16_t overflow;
} Ring;

typedef struct {
    uint16_t rx_bytes;
    uint16_t frames_ok;
    uint16_t err_crc;
    uint16_t err_len;
    uint16_t err_unknown_cmd;
    uint16_t rx_overflow;
    uint16_t uart_errors;
    uint16_t tx_drops;
    uint16_t parser_timeouts;
    uint16_t echo_timeouts;
    uint16_t failsafe_events;
    uint16_t clamp_events;
} Diag;

typedef struct {
    uint32_t boot_ms;
    uint32_t last_valid_frame_ms;
    uint8_t failsafe;
    uint8_t stream_on;
    uint16_t stream_period_ms;
    uint32_t last_telemetry_ms;
    uint16_t flags;
} AppState;

#endif
