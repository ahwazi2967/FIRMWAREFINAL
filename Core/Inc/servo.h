#ifndef SERVO_H
#define SERVO_H

#include "types.h"

void     servo_init(void);
float    servo_clamp_rad(JointId joint, float rad, uint8_t *was_clamped);
uint16_t servo_rad_to_us(JointId joint, float rad);
void     servo_write_us(JointId joint, uint16_t us);

uint8_t  servo_set_target(JointId joint, float rad, uint16_t duration_ms);
uint8_t  servo_set_all_targets(const float rads[NUM_JOINTS], uint16_t duration_ms);
void     servo_update(uint32_t now_ms);
void     servo_stop(void);
void     servo_go_home(uint16_t duration_ms);
float    servo_get_current(JointId joint);
void     servo_get_all_current_mrad(int16_t out[NUM_JOINTS]);
uint8_t  servo_any_moving(void);

#if SERVO_DEBUG
void     servo_debug_write_us(JointId joint, uint16_t us);
uint16_t servo_get_last_us(JointId joint);

#endif

#endif
