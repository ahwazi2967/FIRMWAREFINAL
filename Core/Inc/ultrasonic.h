#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "types.h"

void     ultrasonic_init(void);
void     ultrasonic_task(uint32_t now_ms);
uint16_t ultrasonic_get_mm(void);
uint16_t ultrasonic_get_echo_us(void);
uint16_t ultrasonic_get_timeouts(void);

#endif
