#ifndef RING_H
#define RING_H

#include "types.h"

void        ring_init(Ring *r, uint8_t *storage, uint16_t size);
uint8_t     ring_push(Ring *r, uint8_t byte);
uint8_t     ring_pop(Ring *r, uint8_t *byte);
uint16_t    ring_count(const Ring *r);
uint16_t    ring_free(const Ring *r);

#endif