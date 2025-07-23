#include "PID_Controller.h"
#include <Arduino.h> // For sqrtf()
#include "timing/timing.h"
#include "config.h"
#include "flight_state/Flight_State.h"

namespace PID {

    //Controller:
    float error_roll, error_roll_prev, roll_des_prev, integral_roll, integral_roll_il, integral_roll_ol, integral_roll_prev, integral_roll_prev_il, integral_roll_prev_ol, derivative_roll, roll_PID = 0;
    float error_pitch, error_pitch_prev, pitch_des_prev, integral_pitch, integral_pitch_il, integral_pitch_ol, integral_pitch_prev, integral_pitch_prev_il, integral_pitch_prev_ol, derivative_pitch, pitch_PID = 0;
    float error_yaw, error_yaw_prev, integral_yaw, integral_yaw_prev, derivative_yaw, yaw_PID = 0;
    float GyroX_prev, GyroY_prev, GyroZ_prev = 0.0f;
    
    // Normalized PID outputs
    float m1_command_norm, m2_command_norm, m3_command_norm, m4_command_norm, m5_command_norm, m6_command_norm;
    float s1_command_norm, s2_command_norm, s3_command_norm, s4_command_norm, s5_command_norm, s6_command_norm, s7_command_norm;  

    // Scaled outputs ready for Hardware commands 
    int m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
    int s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;

    void control_angle() {
        //DESCRIPTION: Computes control commands based on state error (angle)
        /*
        * Basic PID control to stablize on angle setpoint based on desired states roll_des, pitch_des, and yaw_des computed in 
        * getDesState(). Error is simply the desired state minus the actual state (ex. roll_des - roll_IMU). Two safety features
        * are implimented here regarding the I terms. The I terms are saturated within specified limits on startup to prevent 
        * excessive buildup. This can be seen by holding the vehicle at an angle and seeing the motors ramp up on one side until
        * they've maxed out throttle...saturating I to a specified limit fixes this. The second feature defaults the I terms to 0
        * if the throttle is at the minimum setting. This means the motors will not start spooling up on the ground, and the I 
        * terms will always start from 0 on takeoff. This function updates the variables roll_PID, pitch_PID, and yaw_PID which
        * can be thought of as 1-D stablized signals. They are mixed to the configuration of the vehicle in controlMixer().
        */

        // First constrain desired values based on Config parameters:
        Flight_State::roll_des =    constrain(Flight_State::roll_des, -1.0, 1.0)*Config::Angle_Mode::maxRoll; //Between -maxRoll and +maxRoll
        Flight_State::pitch_des =   constrain(Flight_State::pitch_des, -1.0, 1.0)*Config::Angle_Mode::maxPitch; //Between -maxPitch and +maxPitch
        Flight_State::yaw_des =     constrain(Flight_State::yaw_des, -1.0, 1.0)*Config::Angle_Mode::maxYaw; //Between -maxYaw and +maxYaw
        
        //Roll
        error_roll = Flight_State::roll_des - Flight_State::roll_IMU;
        integral_roll = integral_roll_prev + error_roll*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_roll = 0;
        }
        integral_roll = constrain(integral_roll, -Config::Angle_Mode::i_limit, Config::Angle_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_roll = Flight_State::GyroX;
        roll_PID = 0.01*(Config::Angle_Mode::Kp_roll_angle*error_roll + Config::Angle_Mode::Ki_roll_angle*integral_roll - Config::Angle_Mode::Kd_roll_angle*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

        //Pitch
        error_pitch = Flight_State::pitch_des - Flight_State::pitch_IMU;
        integral_pitch = integral_pitch_prev + error_pitch*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_pitch = 0;
        }
        integral_pitch = constrain(integral_pitch, -Config::Angle_Mode::i_limit, Config::Angle_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_pitch = Flight_State::GyroY;
        pitch_PID = .01*(Config::Angle_Mode::Kp_pitch_angle*error_pitch + Config::Angle_Mode::Ki_pitch_angle*integral_pitch - Config::Angle_Mode::Kd_pitch_angle*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

        //Yaw, stablize on rate from GyroZ
        error_yaw = Flight_State::yaw_des - Flight_State::GyroZ;
        integral_yaw = integral_yaw_prev + error_yaw*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_yaw = 0;
        }
        integral_yaw = constrain(integral_yaw, -Config::Angle_Mode::i_limit, Config::Angle_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_yaw = (error_yaw - error_yaw_prev)/Timing::dt; 
        yaw_PID = .01*(Config::Angle_Mode::Kp_yaw*error_yaw + Config::Angle_Mode::Ki_yaw*integral_yaw + Config::Angle_Mode::Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

        //Update roll variables
        integral_roll_prev = integral_roll;
        //Update pitch variables
        integral_pitch_prev = integral_pitch;
        //Update yaw variables
        error_yaw_prev = error_yaw;
        integral_yaw_prev = integral_yaw;
    }
    
    void control_rate() {
    //DESCRIPTION: Computes control commands based on state error (rate)
    /*
    * See explanation for controlANGLE(). Everything is the same here except the error is now the desired rate - raw gyro reading.
    */

        // First constrain desired values based on Config parameters:
        Flight_State::roll_des =    constrain(Flight_State::roll_des, -1.0, 1.0)*Config::Rate_Mode::maxRoll; //Between -maxRoll and +maxRoll
        Flight_State::pitch_des =   constrain(Flight_State::pitch_des, -1.0, 1.0)*Config::Rate_Mode::maxPitch; //Between -maxPitch and +maxPitch
        Flight_State::yaw_des =     constrain(Flight_State::yaw_des, -1.0, 1.0)*Config::Rate_Mode::maxYaw; //Between -maxYaw and +maxYaw
            
        //Roll
        error_roll = Flight_State::roll_des - Flight_State::GyroX;
        integral_roll = integral_roll_prev + error_roll*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_roll = 0;
        }
        integral_roll = constrain(integral_roll, -Config::Rate_Mode::i_limit, Config::Rate_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_roll = (error_roll - error_roll_prev)/Timing::dt; 
        roll_PID = .01*(Config::Rate_Mode::Kp_roll_rate*error_roll + Config::Rate_Mode::Ki_roll_rate*integral_roll + Config::Rate_Mode::Kd_roll_rate*derivative_roll); //Scaled by .01 to bring within -1 to 1 range

        //Pitch
        error_pitch = Flight_State::pitch_des - Flight_State::GyroY;
        integral_pitch = integral_pitch_prev + error_pitch*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_pitch = 0;
        }
        integral_pitch = constrain(integral_pitch, -Config::Rate_Mode::i_limit, Config::Rate_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_pitch = (error_pitch - error_pitch_prev)/Timing::dt; 
        pitch_PID = .01*(Config::Rate_Mode::Kp_pitch_rate*error_pitch + Config::Rate_Mode::Ki_pitch_rate*integral_pitch + Config::Rate_Mode::Kd_pitch_rate*derivative_pitch); //Scaled by .01 to bring within -1 to 1 range

        //Yaw, stablize on rate from GyroZ
        error_yaw = Flight_State::yaw_des - Flight_State::GyroZ;
        integral_yaw = integral_yaw_prev + error_yaw*Timing::dt;
        if (Flight_State::thro_des < 0.06f) {   //Don't let integrator build if throttle is too low
            integral_yaw = 0;
        }
        integral_yaw = constrain(integral_yaw, -Config::Rate_Mode::i_limit, Config::Rate_Mode::i_limit); //Saturate integrator to prevent unsafe buildup
        derivative_yaw = (error_yaw - error_yaw_prev)/Timing::dt; 
        yaw_PID = .01*(Config::Rate_Mode::Kp_yaw*error_yaw + Config::Rate_Mode::Ki_yaw*integral_yaw + Config::Rate_Mode::Kd_yaw*derivative_yaw); //Scaled by .01 to bring within -1 to 1 range

        //Update roll variables
        error_roll_prev = error_roll;
        integral_roll_prev = integral_roll;
        GyroX_prev = Flight_State::GyroX;
        //Update pitch variables
        error_pitch_prev = error_pitch;
        integral_pitch_prev = integral_pitch;
        GyroY_prev = Flight_State::GyroY;
        //Update yaw variables
        error_yaw_prev = error_yaw;
        integral_yaw_prev = integral_yaw;
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
    
    void control_mixer() {
        //DESCRIPTION: Mixes scaled commands from PID controller to actuator outputs based on vehicle configuration
        /*
        * Takes roll_PID, pitch_PID, and yaw_PID computed from the PID controller and appropriately mixes them for the desired
        * vehicle configuration. For example on a quadcopter, the left two motors should have +roll_PID while the right two motors
        * should have -roll_PID. Front two should have -pitch_PID and the back two should have +pitch_PID etc... every motor has
        * normalized (0 to 1) thro_des command for throttle control. Can also apply direct unstabilized commands from the transmitter with 
        * roll_passthru, pitch_passthru, and yaw_passthu. mX_command_scaled and sX_command scaled variables are used in scaleCommands() 
        * in preparation to be sent to the motor ESCs and servos.
        * 
        *Relevant variables:
        *thro_des - direct thottle control
        *roll_PID, pitch_PID, yaw_PID - stabilized axis variables
        *roll_passthru, pitch_passthru, yaw_passthru - direct unstabilized command passthrough
        *channel_6_pwm - free auxillary channel, can be used to toggle things with an 'if' statement
        */

        // Main throttle output (motor)
        m1_command_norm = Flight_State::thro_des;

        // Servos
        // Elevator (pitch control)
        s1_command_norm = pitch_PID;  // or pitch_passthru if in manual mode

        // Aileron (roll control)
        s2_command_norm = roll_PID;   // or roll_passthru

        // Rudder (yaw control)
        s3_command_norm = yaw_PID;    // or yaw_passthru
    }

    void scale_commands() {
        // Scale the normalized outputs from the PID controller for PWM servo and ESC outputs
        // For writeMicroseconds:
        m1_command_scaled = constrain(static_cast<int>(1000.0f + m1_command_norm * 1000.0f), 1000, 2000);   // Throttle (motor)
        s1_command_scaled = constrain(static_cast<int>(1500.0f + s1_command_norm * 500.0f), 1000, 2000);    // Elevator (pitch)
        s2_command_scaled = constrain(static_cast<int>(1500.0f + s2_command_norm * 500.0f), 1000, 2000);    // Aileron (roll)
        s3_command_scaled = constrain(static_cast<int>(1500.0f + s3_command_norm * 500.0f), 1000, 2000);    // Rudder (yaw)

        // Constrain the value between 0 and 180 for the servo library
        // Adding 1.0f before * 90 in order to bring servo center to 90 deg.
        // m1_command_scaled = constrain(static_cast<int>(m1_command_norm * 180.0f), 0, 180); // Throttle (motor)
        // s1_command_scaled =  constrain(static_cast<int>((s1_command_norm + 1.0f) * 90.0f), 0, 180); // Elevator (pitch)
        // s2_command_scaled =  constrain(static_cast<int>((s2_command_norm + 1.0f) * 90.0f), 0, 180); // Aileron (roll)
        // s3_command_scaled =  constrain(static_cast<int>((s3_command_norm + 1.0f) * 90.0f), 0, 180); // Rudder (yaw)

    }

}