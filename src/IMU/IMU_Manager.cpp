#include "IMU_Manager.h"
#include "flight_state/Flight_State.h"  // For quaternion and mag fallback

namespace IMU_Manager {
    static IMU imu;

    void init() {
        imu.begin();
        imu.calibrate();
    }

    void update() {
        imu.update();
    }

    // Accelerometer
    float getAccelX() { return imu.getAccelX(); }
    float getAccelY() { return imu.getAccelY(); }
    float getAccelZ() { return imu.getAccelZ(); }

    // Gyroscope
    float getGyroX() { return imu.getGyroX(); }
    float getGyroY() { return imu.getGyroY(); }
    float getGyroZ() { return -imu.getGyroZ(); } // Flip the sign on GyroZ for MPU6050

    // Magnetometer — fallback to Flight_State or return 0
    float getMagX() { return Flight_State::MagX; }
    float getMagY() { return Flight_State::MagY; }
    float getMagZ() { return Flight_State::MagZ; }

    // Quaternion — fallback to Flight_State
    float getQuatW() { return Flight_State::q0; }
    float getQuatX() { return Flight_State::q1; }
    float getQuatY() { return Flight_State::q2; }
    float getQuatZ() { return Flight_State::q3; }
}
