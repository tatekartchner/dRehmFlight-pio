#pragma once
#include "IMU.h"

namespace IMU_Manager {
    void init();
    void update();

    // Accelerometer
    float getAccelX();
    float getAccelY();
    float getAccelZ();

    // Gyroscome
    float getGyroX();
    float getGyroY();
    float getGyroZ();

    // Magnetometer (optional; returns 0 if not available)
    float getMagX();
    float getMagY();
    float getMagZ();

    // Quaternion
    float getQuatW();
    float getQuatX();
    float getQuatY();
    float getQuatZ();
}
