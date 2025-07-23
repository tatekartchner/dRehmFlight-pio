#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <stdint.h>

class Motor_Interface {
public:
    virtual ~Motor_Interface() {}

    virtual void begin() = 0;
    virtual void setThrottle(uint8_t motorIndex, uint16_t value) = 0;  // value in microseconds or normalized
    virtual void arm() = 0;
    virtual void disarm() = 0;
};

#endif
