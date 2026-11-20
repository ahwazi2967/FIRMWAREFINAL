#include "protocol.h"
#include "uart_comm.h"
#include "servo.h"
#include "ultrasonic.h"
#include "app.h"
#include <stddef.h>

static Parser s_parser;

uint8_t crc8_calc(const uint8_t *data, uint16_t len, uint8_t init)
{
    uint8_t crc = init;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8U; b++) {
            if (crc & 0x80U) {
                crc = (uint8_t)((crc << 1) ^ CRC8_POLY);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

void parser_reset(Parser *p)
{
    p->state     = PS_WAIT_H1;
    p->idx       = 0U;
    p->crc       = CRC8_INIT;
    p->frame.len = 0U;
    p->frame.cmd = 0U;
}

uint8_t parser_feed(Parser *p, uint8_t byte, uint32_t now_ms)
{
    p->last_byte_ms = now_ms;

    switch (p->state) {

    case PS_WAIT_H1:
        if (byte == FRAME_H1) {
            p->state = PS_WAIT_H2;
        }
        break;

    case PS_WAIT_H2:
        if (byte == FRAME_H2) {
            p->state = PS_WAIT_LEN;
        } else if (byte != FRAME_H1) {
            p->state = PS_WAIT_H1;       /* bukan header: ulang dari awal */
        }                                /* byte 0xAA lagi: tetap anggap H1 */
        break;

    case PS_WAIT_LEN:
        if (byte > MAX_PAYLOAD) {
            g_diag.err_len++;
            parser_reset(p);
        } else {
            p->frame.len = byte;
            p->crc       = crc8_calc(&byte, 1U, CRC8_INIT);
            p->state     = PS_WAIT_CMD;
        }
        break;

    case PS_WAIT_CMD:
        p->frame.cmd = byte;
        p->crc       = crc8_calc(&byte, 1U, p->crc);
        p->idx       = 0U;
        p->state     = (p->frame.len == 0U) ? PS_WAIT_CRC : PS_PAYLOAD;
        break;

    case PS_PAYLOAD:
        p->frame.payload[p->idx] = byte;
        p->idx++;
        p->crc = crc8_calc(&byte, 1U, p->crc);
        if (p->idx >= p->frame.len) {
            p->state = PS_WAIT_CRC;
        }
        break;

    case PS_WAIT_CRC:
        if (byte == p->crc) {
            p->state = PS_WAIT_H1;
            g_diag.frames_ok++;
            return PARSE_READY;
        }

        g_diag.err_crc++;
        parser_reset(p);
        break;

    default:
        parser_reset(p);
        break;
    }
    return PARSE_NONE;
}

void parser_check_timeout(Parser *p, uint32_t now_ms)
{
    if (p->state != PS_WAIT_H1) {
        if ((now_ms - p->last_byte_ms) >= INTERBYTE_TIMEOUT_MS) {
            g_diag.parser_timeouts++;
            parser_reset(p);
        }
    }
}

void protocol_init(void)
{
	parser_reset(&s_parser);
}

static void put_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)(v >> 8);
}

static void put_i16_le(uint8_t *p, int16_t v)
{
    put_u16_le(p, (uint16_t)v);
}

static uint16_t get_u16_le(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static int16_t get_i16_le(const uint8_t *p)
{
    return (int16_t)get_u16_le(p);
}

static float mrad_to_rad(int16_t mrad)
{
    return (float)mrad / 1000.0f;
}

static uint16_t protocol_build(uint8_t cmd, const uint8_t *payload,
                               uint8_t len, uint8_t *out)
{
    out[0] = FRAME_H1;
    out[1] = FRAME_H2;
    out[2] = len;
    out[3] = cmd;
    for (uint8_t i = 0; i < len; i++) {
        out[4U + i] = payload[i];
    }
    out[4U + len] = crc8_calc(&out[2], (uint16_t)(len + 2U), CRC8_INIT);
    return (uint16_t)(len + 5U);
}

static void send_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    uint8_t buf[MAX_PAYLOAD + 5U];
    uint16_t n = protocol_build(cmd, payload, len, buf);
    uart_send(buf, n);
}

static void send_ack(uint8_t cmd, uint8_t err, const uint8_t *extra, uint8_t extra_len)
{
    uint8_t pl[MAX_PAYLOAD];
    pl[0] = cmd;
    pl[1] = err;
    for (uint8_t i = 0; i < extra_len; i++) {
        pl[2U + i] = extra[i];
    }
    send_frame(CMD_ACK, pl, (uint8_t)(2U + extra_len));
}

static uint16_t status_flags_build(void)
{
    if (servo_any_moving()) {
        g_app.flags |= FLAG_MOVING;
    } else {
        g_app.flags &= (uint16_t)~FLAG_MOVING;
    }
    return g_app.flags;
}

void protocol_send_status(void)
{
    uint8_t pl[14];
    int16_t ang[NUM_JOINTS];

    servo_get_all_current_mrad(ang);
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        put_i16_le(&pl[2U * j], ang[j]);
    }
    put_u16_le(&pl[10], ultrasonic_get_mm());
    put_u16_le(&pl[12], status_flags_build());
    send_frame(CMD_STATUS, pl, 14U);
}

static void handle_ping(const Frame *f)
{
    if (f->len != 0U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    const uint8_t ver[3] = { FW_VERSION_MAJOR, FW_VERSION_MINOR, PROTOCOL_VERSION };
    send_ack(f->cmd, ERR_OK, ver, 3U);
}

static void handle_set_angles(const Frame *f)
{
    if (f->len != 12U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    float rads[NUM_JOINTS];
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        rads[j] = mrad_to_rad(get_i16_le(&f->payload[2U * j]));
    }
    uint16_t dur  = get_u16_le(&f->payload[10]);
    uint8_t  mask = servo_set_all_targets(rads, dur);

    if (mask != 0U) {
        g_app.flags |= FLAG_CLAMPED_LAST;
        g_diag.clamp_events++;
        send_ack(f->cmd, ERR_OK_CLAMPED, &mask, 1U);   /* bit j = sendi j dipotong */
    } else {
        g_app.flags &= (uint16_t)~FLAG_CLAMPED_LAST;
        send_ack(f->cmd, ERR_OK, NULL, 0U);
    }
}

static void handle_home(const Frame *f)
{
    if (f->len != 2U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    servo_go_home(get_u16_le(&f->payload[0]));
    send_ack(f->cmd, ERR_OK, NULL, 0U);
}

static void handle_stop(const Frame *f)
{
    if (f->len != 0U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    servo_stop();
    send_ack(f->cmd, ERR_OK, NULL, 0U);
}

static void handle_get_status(const Frame *f)
{
    if (f->len != 0U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    protocol_send_status();
}

static void handle_set_joint(const Frame *f)
{
    if (f->len != 5U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    uint8_t j = f->payload[0];
    if (j >= NUM_JOINTS) {
        send_ack(f->cmd, ERR_BAD_ARG, NULL, 0U);
        return;
    }
    float    rad = mrad_to_rad(get_i16_le(&f->payload[1]));
    uint16_t dur = get_u16_le(&f->payload[3]);

    if (servo_set_target((JointId)j, rad, dur)) {
        uint8_t mask = (uint8_t)(1U << j);
        g_app.flags |= FLAG_CLAMPED_LAST;
        g_diag.clamp_events++;
        send_ack(f->cmd, ERR_OK_CLAMPED, &mask, 1U);
    } else {
        g_app.flags &= (uint16_t)~FLAG_CLAMPED_LAST;
        send_ack(f->cmd, ERR_OK, NULL, 0U);
    }
}

static void handle_stream(const Frame *f, uint32_t now_ms)
{
    if (f->len != 3U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    uint8_t  en     = f->payload[0];
    uint16_t period = get_u16_le(&f->payload[1]);

    if (en > 1U) {
        send_ack(f->cmd, ERR_BAD_ARG, NULL, 0U);
        return;
    }
    if (period < 20U) {
        period = 20U;                       /* batas bawah dari dokumen */
    }
    g_app.stream_on         = en;
    g_app.stream_period_ms  = period;
    g_app.last_telemetry_ms = now_ms;
    send_ack(f->cmd, ERR_OK, NULL, 0U);
}

static void protocol_send_diag(void)
{
    uint8_t pl[24];
    const uint16_t v[12] = {
        g_diag.rx_bytes,        g_diag.frames_ok,        g_diag.err_crc,
        g_diag.err_len,         g_diag.err_unknown_cmd,  g_diag.rx_overflow,
        g_diag.uart_errors,     g_diag.tx_drops,         g_diag.parser_timeouts,
        g_diag.echo_timeouts,   g_diag.failsafe_events,  g_diag.clamp_events
    };
    for (uint8_t i = 0; i < 12U; i++) {
        put_u16_le(&pl[2U * i], v[i]);
    }
    send_frame(CMD_DIAG, pl, 24U);
}

static void handle_get_diag(const Frame *f)
{
    if (f->len != 0U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    protocol_send_diag();
}

#if SERVO_DEBUG
static void handle_debug_us(const Frame *f)
{
    if (f->len != 3U) {
        send_ack(f->cmd, ERR_BAD_LEN, NULL, 0U);
        return;
    }
    uint8_t j = f->payload[0];
    if (j >= NUM_JOINTS) {
        send_ack(f->cmd, ERR_BAD_ARG, NULL, 0U);
        return;
    }
    uint16_t us   = get_u16_le(&f->payload[1]);
    uint16_t last = servo_get_last_us((JointId)j);
    uint8_t  out[2];

    if (us != 0U) {                                    /* us = 0 berarti hanya bertanya */
        uint16_t diff = (us > last) ? (uint16_t)(us - last) : (uint16_t)(last - us);
        if (diff > DEBUG_STEP_MAX_US) {
            put_u16_le(out, last);
            send_ack(f->cmd, ERR_BAD_ARG, out, 2U);    /* lompatan terlalu besar: ditolak */
            return;
        }
        servo_debug_write_us((JointId)j, us);
        last = servo_get_last_us((JointId)j);          /* nilai yang benar-benar dipakai */
    }
    put_u16_le(out, last);
    send_ack(f->cmd, ERR_OK, out, 2U);
}
#endif

static void protocol_dispatch(const Frame *f, uint32_t now_ms)
{
    g_app.last_valid_frame_ms = now_ms;
    g_app.failsafe = 0U;
    g_app.flags &= (uint16_t)~(FLAG_FAILSAFE | FLAG_BOOT_FRESH);

    switch (f->cmd) {
    case CMD_PING:        handle_ping(f);       break;
    case CMD_SET_ANGLES:  handle_set_angles(f); break;
    case CMD_HOME:        handle_home(f);       break;
    case CMD_STOP:        handle_stop(f);       break;
    case CMD_GET_STATUS:  handle_get_status(f); break;
    case CMD_SET_JOINT:   handle_set_joint(f);        break;
    case CMD_STREAM:      handle_stream(f, now_ms);   break;
    case CMD_GET_DIAG:    handle_get_diag(f);         break;
	#if SERVO_DEBUG
    	case CMD_DEBUG_US:    handle_debug_us(f);         break;
	#endif
    default:
        g_diag.err_unknown_cmd++;
        send_ack(f->cmd, ERR_UNKNOWN_CMD, NULL, 0U);
        break;
    }
}

void protocol_poll(uint32_t now_ms)
{
    uint8_t b;
    uint8_t count = 0U;

    while ((count < 64U) && uart_read_byte(&b)) {
        count++;
        if (parser_feed(&s_parser, b, now_ms) == PARSE_READY) {
            protocol_dispatch(&s_parser.frame, now_ms);
        }
    }
    parser_check_timeout(&s_parser, now_ms);
}

