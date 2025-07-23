#include "PWM_Motor.h"

PWM_Motor::PWM_Motor(const uint8_t* pins, uint8_t motorCount)
    : motorPins(pins), count(motorCount), isArmed(false) {}

void PWM_Motor::begin() {
    for (uint8_t i = 0; i < count; ++i) {
        servos[i].attach(motorPins[i]);
        servos[i].writeMicroseconds(1000); // ESC idle
    }
}

void PWM_Motor::setThrottle(uint8_t motorIndex, uint16_t value) {
    if (motorIndex < count && isArmed) {
        servos[motorIndex].writeMicroseconds(value);  // Proper servo-style PWM
    }
}

void PWM_Motor::arm() {
    isArmed = true;
}

void PWM_Motor::disarm() {
    isArmed = false;
    for (uint8_t i = 0; i < count; ++i) {
        servos[i].writeMicroseconds(1000);  // ESC idle
    }
}
