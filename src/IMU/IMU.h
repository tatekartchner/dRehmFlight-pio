#pragma once
#include <MPU6050_light.h>
#include <Wire.h>

class IMU {
public:
    IMU(TwoWire &wire = Wire);
    void begin();
    void calibrate();
    void update();

    float getAccelX() const;
    float getAccelY() const;
    float getAccelZ() const;

    float getGyroX() const;
    float getGyroY() const;
    float getGyroZ() const;

private:
    MPU6050 mpu;

    // Filter constants
    static constexpr float B_accel = 0.05f;
    static constexpr float B_gyro = 0.05f;

    // Previous values for filtering
    float AccX_prev = 0, AccY_prev = 0, AccZ_prev = 0;
    float GyroX_prev = 0, GyroY_prev = 0, GyroZ_prev = 0;

    // Bias corrections
    float AccErrorX = 0, AccErrorY = 0, AccErrorZ = 0;
    float GyroErrorX = 0, GyroErrorY = 0, GyroErrorZ = 0;

    // Filtered values
    float AccX = 0, AccY = 0, AccZ = 0;
    float GyroX = 0, GyroY = 0, GyroZ = 0;

    void applyFilters(float rawAx, float rawAy, float rawAz,
                      float rawGx, float rawGy, float rawGz);
};
