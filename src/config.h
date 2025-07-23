#pragma once

#include <Arduino.h>

//========================================================================================================================//
//                                                 USER-SPECIFIED DEFINES                                                 //                                                                 
//========================================================================================================================//

//Uncomment only one receiver type
//#define USE_PWM_RX
//#define USE_PPM_RX
//#define USE_SBUS_RX
//#define USE_DSM_RX
#define USE_CRSF_RX

//Uncomment only one IMU
#define USE_MPU6050_I2C //Default
//#define USE_MPU9250_SPI

//Uncomment only one Motor type
#define USE_PWM_Motor

//Uncomment only one full scale gyro range (deg/sec)
#define GYRO_250DPS //Default
//#define GYRO_500DPS
//#define GYRO_1000DPS
//#define GYRO_2000DPS


//Uncomment only one full scale accelerometer range (G's)
#define ACCEL_2G //Default
//#define ACCEL_4G
//#define ACCEL_8G
//#define ACCEL_16G



//========================================================================================================================//
//                                              PIN DEFINITIONS and HARDCODED VALUES                                       //
//========================================================================================================================//



namespace Config {
    constexpr uint8_t num_channels = 6;

    // Failsafe PWM values
    constexpr uint16_t failsafeValues[] = {
        1500,  // Channel 0 - roll
        1500,  // Channel 1 - pitch
        1000,  // Channel 2 - throttle
        1500,  // Channel 3 - yaw
        2000,  // Channel 4 - gear
        1000   // Channel 5 - aux1
        // Add more as needed
    };
    // Count the number of channels so that we don't index out of the array later.
    constexpr uint8_t failsafeChannelCount = sizeof(failsafeValues) / sizeof(uint16_t);

    // Filter parameters - Defaults tuned for 2kHz loop rate; Do not touch unless you know what you are doing
    constexpr float B_madgwick = 0.04;  //Madgwick filter parameter
    constexpr float B_accel = 0.14;     //Accelerometer LP filter paramter, (MPU6050 default: 0.14. MPU9250 default: 0.2)
    constexpr float B_gyro = 0.1;       //Gyro LP filter paramter, (MPU6050 default: 0.1. MPU9250 default: 0.17)
    constexpr float B_mag = 1.0;        //Magnetometer LP filter parameter
    
    namespace Angle_Mode {
        //ANGLE controller parameters (take note of defaults before modifying!): 
        constexpr float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
        constexpr float maxRoll = 30.0;     //Max roll angle in degrees for angle mode (maximum ~70 degrees)
        constexpr float maxPitch = 30.0;    //Max pitch angle in degrees for angle mode (maximum ~70 degrees)
        constexpr float maxYaw = 160.0;     //Max yaw rate in deg/sec (default 160)

        constexpr float Kp_roll_angle = 0.2;    //Roll P-gain - angle mode 
        constexpr float Ki_roll_angle = 0.3;    //Roll I-gain - angle mode
        constexpr float Kd_roll_angle = 0.05;   //Roll D-gain - angle mode (has no effect on controlANGLE2)
        constexpr float B_loop_roll = 0.9;      //Roll damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
        constexpr float Kp_pitch_angle = 0.2;   //Pitch P-gain - angle mode
        constexpr float Ki_pitch_angle = 0.3;   //Pitch I-gain - angle mode
        constexpr float Kd_pitch_angle = 0.05;  //Pitch D-gain - angle mode (has no effect on controlANGLE2)
        constexpr float B_loop_pitch = 0.9;     //Pitch damping term for controlANGLE2(), lower is more damping (must be between 0 to 1)
    
        constexpr float Kp_yaw = 0.3;           //Yaw P-gain
        constexpr float Ki_yaw = 0.05;          //Yaw I-gain
        constexpr float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)

    }

    namespace Rate_Mode {
        //Rate controller parameters (take note of defaults before modifying!): 
        constexpr float i_limit = 25.0;     //Integrator saturation level, mostly for safety (default 25.0)
        constexpr float maxRoll = 360;     //Max roll deg/sec for rate mode (default 30)
        constexpr float maxPitch = 360;    //Max pitch deg/sec for rate mode (default 30)
        constexpr float maxYaw = 360.0;     //Max yaw rate in deg/sec (default 160)
  
        constexpr float Kp_roll_rate = 0.30;    //Roll P-gain         
        constexpr float Ki_roll_rate = 0.1;     //Roll I-gain         
        constexpr float Kd_roll_rate = 0.0002;  //Roll D-gain (be careful when increasing too high, motors will begin to overheat!)
        constexpr float Kp_pitch_rate = 0.30;   //Pitch P-gain
        constexpr float Ki_pitch_rate = 0.1;    //Pitch I-gain
        constexpr float Kd_pitch_rate = 0.0002; //Pitch D-gain (be careful when increasing too high, motors will begin to overheat!)
        constexpr float Kp_yaw = 0.3;           //Yaw P-gain
        constexpr float Ki_yaw = 0.05;          //Yaw I-gain
        constexpr float Kd_yaw = 0.00015;       //Yaw D-gain (be careful when increasing too high, motors will begin to overheat!)
    }

    // Serial for CRSF receiver input
    // If using CRSF, connect to pins 16 & 17 (Serial4)
    inline HardwareSerial& crsf_Serial = Serial4;
    //Note: If using SBUS, connect to pin 21 (RX5). If using DSM, connect to pin 15 (RX3).
    constexpr int PPM_Pin = 23;
    constexpr uint8_t num_DSM_channels = 6; //If using DSM RX, change this to match the number of transmitter channels you have 


    //NOTE: Pin 13 is reserved for onboard LED, pins 18 and 19 are reserved for the MPU6050 IMU for default setup
    // Pins for PWM receiver input (may be wrong)
    constexpr int ch1Pin = 15; // Throttle
    constexpr int ch2Pin = 16; // Ail
    constexpr int ch3Pin = 17; // Ele
    constexpr int ch4Pin = 20; // Rudd
    constexpr int ch5Pin = 21; // Gear (Throttle cut)
    constexpr int ch6Pin = 22; // Aux1 (Free aux channel)

    // OneShot125 ESC pin outputs:
    constexpr int m1Pin = 0;
    constexpr int m2Pin = 1;
    constexpr int m3Pin = 2;
    constexpr int m4Pin = 3;
    constexpr int m5Pin = 4;
    // constexpr int m6Pin = 5;

    // PWM servo or ESC outputs:
    constexpr int esc_Pin = 5;
    constexpr int servo1Pin = 6;
    constexpr int servo2Pin = 7;
    constexpr int servo3Pin = 8;
    constexpr int servo4Pin = 9;
    constexpr int servo5Pin = 10;
    constexpr int servo6Pin = 11;
    constexpr int servo7Pin = 12;

    // Servo min and max pulse widths
    constexpr int servo_min_us = 900;
    constexpr int servo_max_us = 2100;

}
