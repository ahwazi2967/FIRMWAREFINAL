#ifndef PROTOCOL_H
#define PROTOCOL_H

#include "types.h"

#define PARSE_NONE   0U
#define PARSE_READY  1U

uint8_t crc8_calc(const uint8_t *data, uint16_t len, uint8_t init);
void    parser_reset(Parser *p);
uint8_t parser_feed(Parser *p, uint8_t byte, uint32_t now_ms);
void    parser_check_timeout(Parser *p, uint32_t now_ms);

void    protocol_init(void);
void    protocol_poll(uint32_t now_ms);
void    protocol_send_status(void);

#endif
