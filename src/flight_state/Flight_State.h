#pragma once
#include <stdint.h>
#include "config.h"

namespace Flight_State {
    // This array stores the values from the receiver
    extern float channel_pwm[Config::num_channels];
    extern float channel_pwm_prev[Config::num_channels];

    extern float roll_IMU, pitch_IMU, yaw_IMU;
    extern float AccX, AccY, AccZ;
    extern float GyroX, GyroY, GyroZ;
    extern float MagX, MagY, MagZ;
    extern float q0, q1, q2, q3;

    extern float thro_des, roll_des, pitch_des, yaw_des;
    extern float roll_passthru, pitch_passthru, yaw_passthru;

    extern bool armed;

    void failsafe_values();

    void reset();

    void update_from_raw_input(const uint16_t* raw_inputs);

    void get_des_values();

    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);

    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

    float invSqrt(float x);

}
