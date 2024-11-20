#include "robot_message.h"

RobotMessage_t encode_message(uint8_t type, ...){
    RobotMessage_t out_message = {0};
    va_list args;
    va_start(args, type);

    out_message.type = type;
    switch(type){
        case ALERT:
            uint32_t alert_code = (uint16_t) va_arg(args, int);
            out_message.value = alert_code & 0xFFF;
            break;

        case INTENT:
            uint8_t cmd = (uint8_t) va_arg(args, int);
            uint8_t val = (uint8_t) va_arg(args, int);
            out_message.value = ((cmd & 0xF) << 8) | (val & 0xFF);
            break;

        case MOTION_REQUEST:
            uint8_t motor = (uint8_t) va_arg(args, int);
            uint8_t accel = (uint8_t) va_arg(args, int);
            uint8_t speed = (uint8_t) va_arg(args, int);
            out_message.value = ((motor & 1) << 11) | ((accel & 1) << 10) | (speed & 0xFF);
            break;

        case 3: // MOTOR RESPONSE
        case 4: // NAV REQUEST
        case 5: // NAV RESPONSE
        case 6: // ENC REQUEST
        case 7: // ENC RESPONSE
        default:
            out_message.value = 0;
    }

    return out_message;
}