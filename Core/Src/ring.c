#include "ring.h"

void ring_init(Ring *r, uint8_t *storage, uint16_t size) {
    r->buf          = storage;
    r->size         = size;
    r->head         = 0;
    r->tail         = 0;
    r->overflow    = 0;
}

uint8_t ring_push(Ring *r, uint8_t byte) {
    uint16_t next = (uint16_t)((r->head + 1U) % r->size);
    if (next == r->tail) {
        r->overflow++;
        return 0U;
    }
    r->buf[r->head] = byte;
    r->head = next;
    return 1U;
}

uint8_t ring_pop(Ring *r, uint8_t *byte) {
    if (r->head == r->tail) {
        return 0U;
    }
    *byte  = r->buf[r->tail];
    r->tail = (uint16_t)((r->tail + 1U) % r->size);
    return 1U;
}

uint16_t ring_count(const Ring *r) {
    return (uint16_t)((r->head + r->size - r->tail) % r->size);
}

uint16_t ring_free(const Ring *r) {
    return (uint16_t)(r->size - 1U - ring_count(r));
}

