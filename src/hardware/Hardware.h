#pragma once
#include <Servo.h>
#include "config.h"

namespace Hardware {
    extern Servo servo1, servo2, servo3, servo4, servo5, servo6, servo7;
    extern Servo esc;

    void set_up_Servos();

    void calibrate_esc();

    void command_hardware();
}
