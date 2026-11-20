#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>

#define NUM_JOINTS 5U
#define SERVO_UPDATE_MS 20U
#define PWM_PERIOD_US 20000U
#define US_ABS_MIN 500U
#define US_ABS_MAX 2500U

#define FRAME_H1                0xAAU
#define FRAME_H2                0x55U
#define MAX_PAYLOAD             32U
#define CRC8_POLY               0x07U
#define CRC8_INIT               0x00U

#define RX_RING_SIZE            256U
#define TX_RING_SIZE            256U
#define INTERBYTE_TIMEOUT_MS    50U
#define CMD_TIMEOUT_MS          500U
#define HEARTBEAT_MS        	500U

#define ULTRA_PERIOD_MS         60U
#define ULTRA_TIMEOUT_MS        40U
#define ULTRA_MIN_MM            20U
#define ULTRA_MAX_MM            4000U
#define MEDIAN_WINDOW           5U
#define DIST_INVALID            0xFFFFU

#define TELEMETRY_DEFAULT_MS    50U

#define FW_VERSION_MAJOR        0U
#define FW_VERSION_MINOR        1U
#define PROTOCOL_VERSION        1U

#define SERVO_DEBUG             1U
#define DEBUG_STEP_MAX_US		150U
#define SERVO_ARRIVE_EPS_RAD 0.0005f

typedef enum {
    JOINT_BASE = 0,
    JOINT_ARM1,
    JOINT_ARM2,
    JOINT_ARM3,
    JOINT_GRIP
} JointId;

typedef enum {
    CMD_PING            = 0x01,
    CMD_SET_ANGLES      = 0x02,
    CMD_HOME            = 0x03,
    CMD_STOP            = 0x04,
    CMD_GET_STATUS      = 0x05,
    CMD_STREAM          = 0x06,
    CMD_GET_DIAG        = 0x07,
    CMD_SET_JOINT       = 0x08,
	#if SERVO_DEBUG
	    CMD_DEBUG_US        = 0x70,
	#endif
    CMD_ACK             = 0x81,
    CMD_STATUS          = 0x82,
    CMD_DIAG            = 0x83
} CmdCode;

typedef enum {
    ERR_OK          = 0x00,
    ERR_OK_CLAMPED  = 0x01,
    ERR_BAD_LEN     = 0x10,
    ERR_UNKNOWN_CMD = 0x11,
    ERR_BAD_ARG     = 0x12,
} ErrCode;

typedef enum {
    PS_WAIT_H1,
    PS_WAIT_H2,
    PS_WAIT_LEN,
    PS_WAIT_CMD,
    PS_PAYLOAD,
    PS_WAIT_CRC
} ParserState;

typedef enum {
    ULTRA_IDLE,
    ULTRA_WAITING
} UltraState;

#define FLAG_FAILSAFE        (1U << 0)
#define FLAG_MOVING          (1U << 1)
#define FLAG_CLAMPED_LAST    (1U << 2)
#define FLAG_ECHO_VALID      (1U << 3)
#define FLAG_RX_OVERFLOW     (1U << 4)
#define FLAG_BOOT_FRESH      (1U << 5)

#endif
