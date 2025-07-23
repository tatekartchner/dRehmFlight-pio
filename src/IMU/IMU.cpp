#include "IMU.h"

IMU::IMU(TwoWire &wire) : mpu(wire) {}

void IMU::begin() {
    Wire.begin();
    Wire.setClock(1000000); //Note this is 2.5 times the spec sheet 400 kHz max...
    // Wire.setClock(400000);  // Standard "fast mode" - Use if there are issues with IMU stability.

    byte status = mpu.begin();
    if (status != 0) {
        // Optional: Add Serial.println or error LED indicator
    }
    delay(1000);  // Give sensor time to stabilize
    calibrate();
}

void IMU::calibrate() {
    mpu.calcOffsets(true, true);  // Automatically calibrates accel & gyro offsets

    // If needed, adjust your own error values here
    AccErrorX = 0.0f;
    AccErrorY = 0.0f;
    AccErrorZ = 0.0f;

    GyroErrorX = 0.0f;
    GyroErrorY = 0.0f;
    GyroErrorZ = 0.0f;
}

void IMU::update() {
    mpu.update();

    // Get scaled values (already in g's and deg/sec)
    float rawAx = mpu.getAccX();
    float rawAy = mpu.getAccY();
    float rawAz = mpu.getAccZ();

    float rawGx = mpu.getGyroX();
    float rawGy = mpu.getGyroY();
    float rawGz = mpu.getGyroZ();

    applyFilters(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
}

void IMU::applyFilters(float rawAx, float rawAy, float rawAz,
                       float rawGx, float rawGy, float rawGz) {
    // Bias correction
    rawAx -= AccErrorX;
    rawAy -= AccErrorY;
    rawAz -= AccErrorZ;

    rawGx -= GyroErrorX;
    rawGy -= GyroErrorY;
    rawGz -= GyroErrorZ;

    // Low-pass filter
    AccX = (1.0f - B_accel) * AccX_prev + B_accel * rawAx;
    AccY = (1.0f - B_accel) * AccY_prev + B_accel * rawAy;
    AccZ = (1.0f - B_accel) * AccZ_prev + B_accel * rawAz;

    GyroX = (1.0f - B_gyro) * GyroX_prev + B_gyro * rawGx;
    GyroY = (1.0f - B_gyro) * GyroY_prev + B_gyro * rawGy;
    GyroZ = (1.0f - B_gyro) * GyroZ_prev + B_gyro * rawGz;

    // Store for next filter cycle
    AccX_prev = AccX;
    AccY_prev = AccY;
    AccZ_prev = AccZ;

    GyroX_prev = GyroX;
    GyroY_prev = GyroY;
    GyroZ_prev = GyroZ;
}

// Getters
float IMU::getAccelX() const { return AccX; }
float IMU::getAccelY() const { return AccY; }
float IMU::getAccelZ() const { return AccZ; }

float IMU::getGyroX() const { return GyroX; }
float IMU::getGyroY() const { return GyroY; }
float IMU::getGyroZ() const { return GyroZ; }
