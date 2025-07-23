#ifndef MOTOR_FACTORY_H
#define MOTOR_FACTORY_H

#include "config.h"
#include "Motor_Interface.h"

#ifdef USE_PWM_Motor
#include "PWM_Motor.h"
#endif

static Motor_Interface* createMotorController() {
#ifdef USE_PWM_Motor
    static const uint8_t motorPins[] = {5};  // Replace with your actual motor PWM pins
    return new PWM_Motor(motorPins, sizeof(motorPins) / sizeof(motorPins[0]));
#else
    #error "No motor type defined in config.h"
#endif
}

#endif
