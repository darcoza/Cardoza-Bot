#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
// #include "freertos/queue.h"

enum RobotMessageType {
    ALERT,
    INTENT,
    MOTION_REQUEST,
    MOTION_RESPONSE,
    NAV_REQUEST,
    NAV_RESPONSE,
    ENC_REQUEST,
    ENC_RESPONSE    
};

enum RobotMessageAlertType{
    IO_DISCONNECTED,
    IO_CONNECTED
};

enum RobotMessageIntentType{
    STOP,
    DRIVE_FORWARD,
    DRIVE_REVERSE,
    TURN_LEFT,
    TURN_RIGHT,
    ENABLE_SPIN,
    DISABLE_SPIN,
    TANK_DRIVE_LEFT,
    TANK_DRIVE_RIGHT
};

typedef struct {
    uint8_t type : 4;
    uint16_t value : 12;
} RobotMessage_t;

RobotMessage_t encode_message(uint8_t, ...);