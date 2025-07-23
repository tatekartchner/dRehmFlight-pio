#ifndef PWM_MOTOR_H
#define PWM_MOTOR_H

#include "Motor_Interface.h"
#include <Servo.h>

class PWM_Motor : public Motor_Interface {
public:
    PWM_Motor(const uint8_t* pins, uint8_t motorCount);
    void begin() override;
    void setThrottle(uint8_t motorIndex, uint16_t value) override;
    void arm() override;
    void disarm() override;

private:
    Servo servos[8];  // Max 8 motors
    const uint8_t* motorPins;
    uint8_t count;
    bool isArmed;
};

#endif
