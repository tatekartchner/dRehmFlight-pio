#pragma once


namespace PID {

    // //Normalized desired state:
    // extern float thro_des, roll_des, pitch_des, yaw_des;
    // extern float roll_passthru, pitch_passthru, yaw_passthru;

    //Mixer
    extern int m1_command_scaled, m2_command_scaled, m3_command_scaled, m4_command_scaled, m5_command_scaled, m6_command_scaled;
    extern int s1_command_scaled, s2_command_scaled, s3_command_scaled, s4_command_scaled, s5_command_scaled, s6_command_scaled, s7_command_scaled;

    //Enable PID value printing by making extern:
    extern float roll_PID, pitch_PID, yaw_PID;

    void Madgwick(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float invSampleFreq);

    void Madgwick6DOF(float gx, float gy, float gz, float ax, float ay, float az, float invSampleFreq);

    void control_angle();

    void control_rate();

    void control_mixer();

    void scale_commands();

    float invSqrt(float x);


}