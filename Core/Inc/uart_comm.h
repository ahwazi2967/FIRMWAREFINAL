#ifndef UART_COMM_H
#define UART_COMM_H

#include "types.h"

void     uart_comm_init(void);
uint8_t  uart_send(const uint8_t *data, uint16_t len);
uint8_t  uart_read_byte(uint8_t *byte);
uint16_t uart_rx_available(void);

#endif