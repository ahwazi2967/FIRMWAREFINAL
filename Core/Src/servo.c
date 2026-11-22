#include "servo.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

/* timer, channel, rad_a, us_a, rad_b, us_b,rad_min, rad_max, home_rad, max_speed_rad_s        */
static const ServoConfig s_cfg[NUM_JOINTS] = {
    /* BASE */ { &htim2, TIM_CHANNEL_1, 0.0f, 1500.0f, 0.7854f, 2000.0f, -1.523f, 1.523f, 0.0f, 1.0f },
    /* ARM1 */ { &htim2, TIM_CHANNEL_2, 0.0f, 1500.0f, 0.7854f, 2000.0f, -1.523f, 1.523f, 0.0f, 1.0f },
    /* ARM2 */ { &htim2, TIM_CHANNEL_3, 0.0f, 1500.0f, 0.7854f, 2000.0f, -1.319f, 1.523f, 0.0f, 1.0f },
    /* ARM3 */ { &htim2, TIM_CHANNEL_4, 0.0f, 1500.0f, 0.7854f, 2000.0f, -1.523f, 1.523f, 0.0f, 1.0f },
    /* GRIP */ { &htim3, TIM_CHANNEL_1, 0.0f, 1500.0f, 0.7854f, 2000.0f, 0.0f, 1.052f, 0.0f, 1.0f }
};

static ServoState s_state[NUM_JOINTS];

float servo_clamp_rad(JointId joint, float rad, uint8_t *was_clamped)
{
    const ServoConfig *c = &s_cfg[joint];
    *was_clamped = 0U;

    if (rad != rad) {                      
        *was_clamped = 1U;
        return s_state[joint].current_rad;
    }
    if (rad < c->rad_min) { *was_clamped = 1U; return c->rad_min; }
    if (rad > c->rad_max) { *was_clamped = 1U; return c->rad_max; }
    return rad;
}

uint16_t servo_rad_to_us(JointId joint, float rad)
{
    const ServoConfig *c = &s_cfg[joint];
    float span = c->rad_b - c->rad_a;
    float us;

    if (span == 0.0f) {
        us = c->us_a;                          /* cegah pembagian nol */
    } else {
        us = c->us_a + (rad - c->rad_a) * (c->us_b - c->us_a) / span;
    }
    if (us < (float)US_ABS_MIN) us = (float)US_ABS_MIN;
    if (us > (float)US_ABS_MAX) us = (float)US_ABS_MAX;
    return (uint16_t)(us + 0.5f);              /* dibulatkan */
}

void servo_write_us(JointId joint, uint16_t us)
{
    __HAL_TIM_SET_COMPARE(s_cfg[joint].htim, s_cfg[joint].channel, us);
    s_state[joint].last_us = us;
}

void servo_init(void)
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        float home = s_cfg[j].home_rad;
        s_state[j].current_rad = home;
        s_state[j].target_rad  = home;
        s_state[j].start_rad   = home;
        s_state[j].start_ms    = 0U;
        s_state[j].duration_ms = 0U;
        s_state[j].moving      = 0U;
        servo_write_us((JointId)j, servo_rad_to_us((JointId)j, home));
    }
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        HAL_TIM_PWM_Start(s_cfg[j].htim, s_cfg[j].channel);
    }
}

static uint8_t set_target_at(JointId joint, float rad,
                             uint16_t duration_ms, uint32_t now)
{
    uint8_t clamped;
    float target = servo_clamp_rad(joint, rad, &clamped);
    ServoState *s = &s_state[joint];

    s->start_rad   = s->current_rad;      /* gerak mulai dari posisi sekarang */
    s->target_rad  = target;
    s->start_ms    = now;
    s->duration_ms = duration_ms;
    s->moving      = (target != s->current_rad) ? 1U : 0U;
    return clamped;
}

#if SERVO_DEBUG
void servo_debug_write_us(JointId joint, uint16_t us)
{
    if (us < US_ABS_MIN) us = US_ABS_MIN;
    if (us > US_ABS_MAX) us = US_ABS_MAX;
    servo_write_us(joint, us);
}

uint16_t servo_get_last_us(JointId joint)
{
    return s_state[joint].last_us;
}
#endif

uint8_t servo_set_target(JointId joint, float rad, uint16_t duration_ms)
{
    return set_target_at(joint, rad, duration_ms, HAL_GetTick());
}

uint8_t servo_set_all_targets(const float rads[NUM_JOINTS], uint16_t duration_ms)
{
    uint32_t now = HAL_GetTick();
    uint8_t mask = 0U;
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (set_target_at((JointId)j, rads[j], duration_ms, now)) {
            mask |= (uint8_t)(1U << j);
        }
    }
    return mask;
}

void servo_go_home(uint16_t duration_ms)
{
    uint32_t now = HAL_GetTick();
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        set_target_at((JointId)j, s_cfg[j].home_rad, duration_ms, now);
    }
}

void servo_update(uint32_t now_ms)
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        ServoState *s = &s_state[j];
        if (!s->moving) {
            continue;
        }

        float max_step = s_cfg[j].max_speed_rad_s *
                         ((float)SERVO_UPDATE_MS / 1000.0f);
        float step = s->target_rad - s->current_rad;      /* tanpa durasi: langsung ke target */

        if (s->duration_ms > 0U) {
        	int32_t  dt      = (int32_t)(now_ms - s->start_ms);
        	uint32_t elapsed = (dt > 0) ? (uint32_t)dt : 0U;
            if (elapsed < s->duration_ms) {
                float frac   = (float)elapsed / (float)s->duration_ms;
                float wanted = s->start_rad +
                               (s->target_rad - s->start_rad) * frac;
                step = wanted - s->current_rad;
            }
        }

        if (step >  max_step) step =  max_step;           /* pembatas kecepatan */
        if (step < -max_step) step = -max_step;

        s->current_rad += step;

        float err = s->target_rad - s->current_rad;
        if (err < SERVO_ARRIVE_EPS_RAD && err > -SERVO_ARRIVE_EPS_RAD) {
            s->current_rad = s->target_rad;
            s->moving = 0U;
        }
        servo_write_us((JointId)j, servo_rad_to_us((JointId)j, s->current_rad));
    }
}

void servo_stop(void)
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        s_state[j].target_rad = s_state[j].current_rad;
        s_state[j].start_rad  = s_state[j].current_rad;
        s_state[j].moving     = 0U;
    }
}

float servo_get_current(JointId joint)
{
    return s_state[joint].current_rad;
}

void servo_get_all_current_mrad(int16_t out[NUM_JOINTS])
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        float m = s_state[j].current_rad * 1000.0f;
        out[j] = (int16_t)(m >= 0.0f ? m + 0.5f : m - 0.5f);   /* dibulatkan */
    }
}

uint8_t servo_any_moving(void)
{
    for (uint8_t j = 0; j < NUM_JOINTS; j++) {
        if (s_state[j].moving) {
            return 1U;
        }
    }
    return 0U;
}
