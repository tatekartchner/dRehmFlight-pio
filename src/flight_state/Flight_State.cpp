#include "Flight_State.h"
#include <Arduino.h> // For sqrtf()


namespace Flight_State {

    float roll_IMU = 0, pitch_IMU = 0, yaw_IMU = 0;
    float AccX = 0, AccY = 0, AccZ = 0;
    float GyroX = 0, GyroY = 0, GyroZ = 0;
    float MagX = 0, MagY = 0, MagZ = 0;
    float q0 = 1.0f, q1 = 0, q2 = 0, q3 = 0;

    float thro_des = 0, roll_des = 0, pitch_des = 0, yaw_des = 0; // Normalized values
    float roll_passthru = 0, pitch_passthru = 0, yaw_passthru = 0;
    
    bool armed = false;


    // Initialize the flight state channel array
    float channel_pwm[Config::num_channels]; // Populated at startup with failsafe values
    float channel_pwm_prev[Config::num_channels]; // Populated at startup with failsafe values

    // uint16_t channel_pwm[Config::num_channels]; 
    // uint16_t channel_pwm_prev[Config::num_channels]; 

    struct Initializer {
        Initializer() {
                for (int i = 0; i < Config::num_channels; ++i) {
                channel_pwm[i] = Config::failsafeValues[i];
                channel_pwm_prev[i] = Config::failsafeValues[i];
            }
        }
    };

    static Initializer _init; // This does the initialization of the failsafe values on startup.

    void failsafe_values() {
        // Write all the channel values to their failsafe values
        for (int i = 0; i < Config::num_channels; ++i) {
            channel_pwm[i] = Config::failsafeValues[i];
            channel_pwm_prev[i] = Config::failsafeValues[i];
        }
    }

    void reset() {
        roll_IMU = pitch_IMU = yaw_IMU = 0;
        AccX = AccY = AccZ = 0;
        GyroX = GyroY = GyroZ = 0;
        MagX = MagY = MagZ = 0;
        q0 = 1.0f; q1 = q2 = q3 = 0;
        thro_des = roll_des = pitch_des = yaw_des = 0;
        armed = false;
    }

    void update_from_raw_input(const uint16_t* raw_inputs) {
        //Low-pass the critical commands and update previous values
        float b = 0.7f;
        for (int i = 0; i < Config::num_channels; ++i) {
            // Cast to float for filtering and future normalization
            float input = static_cast<float>(raw_inputs[i]); 
            channel_pwm[i] = (1.0f - b) * channel_pwm_prev[i] + b * input;
            channel_pwm_prev[i] = channel_pwm[i];
        }
        if (channel_pwm[4] >= 1900) { // Use the button to arm the plane
            armed = true;
        } else {
            armed = false;
        }
    }

    void get_des_values() {
        // Normalize the raw PWM values
        thro_des = (channel_pwm[2] - 1000.0)/1000.0; //Between 0 and 1 (THROTTLE)
        roll_des = (channel_pwm[0] - 1500.0)/500.0; //Between -1 and 1 (AILERONS)
        pitch_des = (channel_pwm[1] - 1500.0)/500.0; //Between -1 and 1 (ELEVATOR)
        yaw_des = (channel_pwm[3] - 1500.0)/500.0; //Between -1 and 1 (RUDDER)
        roll_passthru = roll_des/2.0; //Between -0.5 and 0.5
        pitch_passthru = pitch_des/2.0; //Between -0.5 and 0.5
        yaw_passthru = yaw_des/2.0; //Between -0.5 and 0.5
        
        //Constrain within normalized bounds
        roll_passthru = constrain(roll_passthru, -0.5, 0.5);
        pitch_passthru = constrain(pitch_passthru, -0.5, 0.5);
        yaw_passthru = constrain(yaw_passthru, -0.5, 0.5);
        thro_des = constrain(thro_des, 0.0, 1.0); //Between 0 and 1
    }

    void Madgwick(float GyroX, float GyroY, float GyroZ, float AccX, float AccY, float AccZ, float MagX, float MagY, float MagZ, float invSampleFreq) {
        //DESCRIPTION: Attitude estimation through sensor fusion - 9DOF
        /*
        * This function fuses the accelerometer gyro, and magnetometer readings AccX, AccY, AccZ, GyroX, GyroY, GyroZ, MagX, MagY, and MagZ for attitude estimation.
        * Don't worry about the math. There is a tunable parameter B_madgwick in the user specified variable section which basically
        * adjusts the weight of gyro data in the state estimate. Higher beta leads to noisier estimate, lower 
        * beta leads to slower to respond estimate. It is currently tuned for 2kHz loop rate. This function updates the roll_IMU,
        * pitch_IMU, and yaw_IMU variables which are in degrees. If magnetometer data is not available, this function calls Madgwick6DOF() instead.
        */
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float hx, hy;
        float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        //use 6DOF algorithm if MPU6050 is being used
        #if defined USE_MPU6050_I2C 
            Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, invSampleFreq);
            return;
        #endif
        
        //Use 6DOF algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalization)
        if((MagX == 0.0f) && (MagY == 0.0f) && (MagZ == 0.0f)) {
            Madgwick6DOF(GyroX, GyroY, GyroZ, AccX, AccY, AccZ, invSampleFreq);
            return;
        }

        //Convert gyroscope degrees/sec to radians/sec
        float GyroX_rad = GyroX * 0.0174533f;
        float GyroY_rad = GyroY * 0.0174533f;
        float GyroZ_rad = GyroZ * 0.0174533f;

        //Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * GyroX_rad - q2 * GyroY_rad - q3 * GyroZ_rad);
        qDot2 = 0.5f * (q0 * GyroX_rad + q2 * GyroZ_rad - q3 * GyroY_rad);
        qDot3 = 0.5f * (q0 * GyroY_rad - q1 * GyroZ_rad + q3 * GyroX_rad);
        qDot4 = 0.5f * (q0 * GyroZ_rad + q1 * GyroY_rad - q2 * GyroX_rad);

        //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalization)
        if(!((AccX == 0.0f) && (AccY == 0.0f) && (AccZ == 0.0f))) {

            //Normalise accelerometer measurement
            recipNorm = invSqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
            AccX *= recipNorm;
            AccY *= recipNorm;
            AccZ *= recipNorm;

            //Normalise magnetometer measurement
            recipNorm = invSqrt(MagX * MagX + MagY * MagY + MagZ * MagZ);
            MagX *= recipNorm;
            MagY *= recipNorm;
            MagZ *= recipNorm;

            //Auxiliary variables to avoid repeated arithmetic
            _2q0mx = 2.0f * q0 * MagX;
            _2q0my = 2.0f * q0 * MagY;
            _2q0mz = 2.0f * q0 * MagZ;
            _2q1mx = 2.0f * q1 * MagX;
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _2q0q2 = 2.0f * q0 * q2;
            _2q2q3 = 2.0f * q2 * q3;
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            //Reference direction of Earth's magnetic field
            hx = MagX * q0q0 - _2q0my * q3 + _2q0mz * q2 + MagX * q1q1 + _2q1 * MagY * q2 + _2q1 * MagZ * q3 - MagX * q2q2 - MagX * q3q3;
            hy = _2q0mx * q3 + MagY * q0q0 - _2q0mz * q1 + _2q1mx * q2 - MagY * q1q1 + MagY * q2q2 + _2q2 * MagZ * q3 - MagY * q3q3;
            _2bx = sqrtf(hx * hx + hy * hy);
            _2bz = -_2q0mx * q2 + _2q0my * q1 + MagZ * q0q0 + _2q1mx * q3 - MagZ * q1q1 + _2q2 * MagY * q3 - MagZ * q2q2 + MagZ * q3q3;
            _4bx = 2.0f * _2bx;
            _4bz = 2.0f * _2bz;

            //Gradient decent algorithm corrective step
            s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - AccX) + _2q1 * (2.0f * q0q1 + _2q2q3 - AccY) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - MagX) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - MagY) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - MagZ);
            s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - AccX) + _2q0 * (2.0f * q0q1 + _2q2q3 - AccY) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - AccZ) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - MagX) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - MagY) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - MagZ);
            s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - AccX) + _2q3 * (2.0f * q0q1 + _2q2q3 - AccY) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - AccZ) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - MagX) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - MagY) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - MagZ);
            s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - AccX) + _2q2 * (2.0f * q0q1 + _2q2q3 - AccY) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - MagX) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - MagY) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - MagZ);
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            //Apply feedback step
            qDot1 -= Config::B_madgwick * s0;
            qDot2 -= Config::B_madgwick * s1;
            qDot3 -= Config::B_madgwick * s2;
            qDot4 -= Config::B_madgwick * s3;
        }

        //Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        //Normalize quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;
        
        //compute angles - NWU
        roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
        pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
        yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
    }

    void Madgwick6DOF(float GyroX, float GyroY, float GyroZ, float AccX, float AccY, float AccZ, float invSampleFreq) {
        //DESCRIPTION: Attitude estimation through sensor fusion - 6DOF
        /*
        * See description of Madgwick() for more information. This is a 6DOF implimentation for when magnetometer data is not
        * available (for example when using the recommended MPU6050 IMU for the default setup).
        */
        float recipNorm;
        float s0, s1, s2, s3;
        float qDot1, qDot2, qDot3, qDot4;
        float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

        //Convert gyroscope degrees/sec to radians/sec
        float GyroX_rad = GyroX * 0.0174533f;
        float GyroY_rad = GyroY * 0.0174533f;
        float GyroZ_rad = GyroZ * 0.0174533f;

        //Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * GyroX_rad - q2 * GyroY_rad - q3 * GyroZ_rad);
        qDot2 = 0.5f * (q0 * GyroX_rad + q2 * GyroZ_rad - q3 * GyroY_rad);
        qDot3 = 0.5f * (q0 * GyroY_rad - q1 * GyroZ_rad + q3 * GyroX_rad);
        qDot4 = 0.5f * (q0 * GyroZ_rad + q1 * GyroY_rad - q2 * GyroX_rad);

        //Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        if(!((AccX == 0.0f) && (AccY == 0.0f) && (AccZ == 0.0f))) {
            //Normalise accelerometer measurement
            recipNorm = invSqrt(AccX * AccX + AccY * AccY + AccZ * AccZ);
            AccX *= recipNorm;
            AccY *= recipNorm;
            AccZ *= recipNorm;

            //Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0f * q0;
            _2q1 = 2.0f * q1;
            _2q2 = 2.0f * q2;
            _2q3 = 2.0f * q3;
            _4q0 = 4.0f * q0;
            _4q1 = 4.0f * q1;
            _4q2 = 4.0f * q2;
            _8q1 = 8.0f * q1;
            _8q2 = 8.0f * q2;
            q0q0 = q0 * q0;
            q1q1 = q1 * q1;
            q2q2 = q2 * q2;
            q3q3 = q3 * q3;

            //Gradient decent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * AccX + _4q0 * q1q1 - _2q1 * AccY;
            s1 = _4q1 * q3q3 - _2q3 * AccX + 4.0f * q0q0 * q1 - _2q0 * AccY - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * AccZ;
            s2 = 4.0f * q0q0 * q2 + _2q0 * AccX + _4q2 * q3q3 - _2q3 * AccY - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * AccZ;
            s3 = 4.0f * q1q1 * q3 - _2q1 * AccX + 4.0f * q2q2 * q3 - _2q2 * AccY;
            recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); //normalise step magnitude
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            //Apply feedback step
            qDot1 -= Config::B_madgwick * s0;
            qDot2 -= Config::B_madgwick * s1;
            qDot3 -= Config::B_madgwick * s2;
            qDot4 -= Config::B_madgwick * s3;
        }

        //Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * invSampleFreq;
        q1 += qDot2 * invSampleFreq;
        q2 += qDot3 * invSampleFreq;
        q3 += qDot4 * invSampleFreq;

        //Normalise quaternion
        recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        //Compute angles
        roll_IMU = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2)*57.29577951; //degrees
        pitch_IMU = -asin(constrain(-2.0f * (q1*q3 - q0*q2),-0.999999,0.999999))*57.29577951; //degrees
        yaw_IMU = -atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3)*57.29577951; //degrees
    }

    float invSqrt(float x) {
        //Fast inverse sqrt for madgwick filter
        /*
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        y = y * (1.5f - (halfx * y * y));
        return y;
        */
        /*
        //alternate form:
        unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
        float tmp = *(float*)&i;
        float y = tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
        return y;
        */
        return 1.0/sqrtf(x); //Teensy is fast enough to just take the compute penalty lol suck it arduino nano
    }



}
