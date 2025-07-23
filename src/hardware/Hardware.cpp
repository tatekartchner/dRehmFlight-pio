#include "Hardware.h"
#include "PID/PID_Controller.h"

namespace Hardware {
    Servo servo1;
    Servo servo2;
    Servo servo3;
    Servo servo4;
    Servo servo5;
    Servo servo6;
    Servo servo7;
    Servo esc;

    void set_up_Servos() {
        servo1.attach(Config::servo1Pin, 1000, 2000);
        servo2.attach(Config::servo2Pin, 1000, 2000);    
        servo3.attach(Config::servo3Pin, 1000, 2000);
        servo4.attach(Config::servo4Pin, 1000, 2000);
        servo5.attach(Config::servo5Pin, 1000, 2000);
        servo6.attach(Config::servo6Pin, 1000, 2000);
        servo7.attach(Config::servo7Pin, 1000, 2000);  

        esc.attach(Config::esc_Pin);
    }

    void calibrate_esc() {
        // Serial.println("Initializing ESC");
        // Serial.println("Writing High to ESC");
        Hardware::esc.writeMicroseconds(2000);
        delay(1000);
        // Serial.println("Writing Low to ESC");
        Hardware::esc.writeMicroseconds(1000);
        delay(1000);
        // Wait for ESC startup beeps
    }

    void command_hardware() {
        // This function outputs the scaled ESC and servo commands to motors.
        esc.writeMicroseconds(PID::m1_command_scaled); // Throttle (motor)

        servo1.writeMicroseconds(PID::s1_command_scaled); // Elevator (pitch)
        servo2.writeMicroseconds(PID::s2_command_scaled); // Aileron (roll)
        servo3.writeMicroseconds(PID::s3_command_scaled); // Rudder (yaw)
    }

}



