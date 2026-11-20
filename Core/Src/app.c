#include "app.h"
#include "uart_comm.h"
#include "servo.h"
#include "ultrasonic.h"
#include "protocol.h"

Diag     g_diag;
AppState g_app;

static uint32_t t_servo_ms;
static uint32_t t_failsafe_ms;
static uint32_t t_heartbeat_ms;

static uint8_t task_due(uint32_t *last_ms, uint32_t period_ms, uint32_t now)
{
    if ((uint32_t)(now - *last_ms) >= period_ms) {
        *last_ms = now;
        return 1U;
    }
    return 0U;
}

static void failsafe_task(uint32_t now)
{
    if (g_app.failsafe) {
        return;                                    /* sudah aktif */
    }
    if (g_app.flags & FLAG_BOOT_FRESH) {
        return;                                    /* belum pernah ada perintah */
    }
    if ((uint32_t)(now - g_app.last_valid_frame_ms) >= CMD_TIMEOUT_MS) {
        g_app.failsafe = 1U;
        g_app.flags   |= FLAG_FAILSAFE;
        g_diag.failsafe_events++;
        servo_stop();                              /* PWM tetap jalan */
    }
}

static void telemetry_task(uint32_t now)
{
    if (g_app.stream_on &&
        task_due(&g_app.last_telemetry_ms, g_app.stream_period_ms, now)) {
        protocol_send_status();
    }
}

static void heartbeat_task(uint32_t now)
{
#ifdef LED_Pin
    if (task_due(&t_heartbeat_ms, HEARTBEAT_MS, now)) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }
#else
    (void)now;
#endif
}

void app_init(void)
{
    uart_comm_init();
    servo_init();                 /* tulis pulsa home SEBELUM PWM menyala */
    ultrasonic_init();
    protocol_init();

    g_app.stream_period_ms = TELEMETRY_DEFAULT_MS;
}

void app_loop(void)
{
    uint32_t now = HAL_GetTick();

    protocol_poll(now);
    ultrasonic_task(now);

    if (task_due(&t_servo_ms, SERVO_UPDATE_MS, now)) {
        servo_update(now);
    }
    if (task_due(&t_failsafe_ms, SERVO_UPDATE_MS, now)) {
        failsafe_task(now);
    }
    telemetry_task(now);
    heartbeat_task(now);
}
