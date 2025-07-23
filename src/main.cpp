#include <Arduino.h>
#include <Servo.h>
#include <stdint.h>
#include "IMU/IMU_Manager.h"
#include "receiver/Receiver_Manager.h"
// #include "motors/Motor_Factory.h"
#include "flight_state/Flight_State.h"
#include "hardware/Hardware.h"
#include "timing/timing.h"
#include "PID/PID_Controller.h"


//========================================================================================================================//

//Function declarations

void get_IMU_data();

void print_IMU_data();

void get_commands();

void print_commands();

void calibrate_attitude();

void print_attitude();

void print_receiver_input();

void print_scaled_outputs();

void print_PID_values();

//========================================================================================================================//
//                    SETUP for MAIN LOOP                                                                                 //
//========================================================================================================================//
void setup() {
  Timing::begin(); // Set up onboard LED for heartbeat
  Timing::set_loop_freq(2000); // Set loop frequency. All PID values are tuned to 2kHz.

  Serial.begin(115200); // For printing to the serial monitor
  Serial.println("Initializing setup() ");

  Receiver_Manager::init(); // Set up the receiver (Automatically sets baud rate)
  IMU_Manager::init(); // Start I2C communication with the IMU
  Hardware::set_up_Servos(); // Set up the servos and pwm ESC
  Hardware::esc.writeMicroseconds(1000); // Don't let the motor spaz on startup.
  // Uncomment to calibrate the ESC on startup. The motor may spin. PROPS OFF!!
  // Hardware::calibrate_esc();

  // calibrate_attitude(); //Just calls the IMU and PID a bunch to warm up filters before main loop

}

//========================================================================================================================//
//                    MAIN LOOP                                                                                           //
//========================================================================================================================//

void loop() {
  Timing::update(); // Update dt, current_time, prev_time, etc.
  Timing::heartbeat_LED(); // Blink the onboard LED (It's fun!)

  // Serial.print("Failsafe active: "); Serial.println(Receiver_Manager::isFailsafe());
  // Serial.print("dt: "); Serial.println(Timing::dt * 1000000); //Expected 500 microseconds for 2 kHz

  IMU_Manager::update();  // Stores the most recent IMU values
  get_IMU_data();         // Updates variables in Flight_State based on available IMU values

  //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  Flight_State::Madgwick(Flight_State::GyroX, -Flight_State::GyroY, -Flight_State::GyroZ, -Flight_State::AccX, Flight_State::AccY, Flight_State::AccZ, Flight_State::MagY, -Flight_State::MagX, Flight_State::MagZ, Timing::dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
  

  Flight_State::get_des_values(); // Scales receiver input for PID control
  // PID::control_angle();
  PID::control_rate(); //Angle mode doesn't really work for fixed wing planes - Think about doing a barrel role in rate mode.
  PID::control_mixer(); // Combine PID parameters to create the thr, rud, ail, ele commands
  PID::scale_commands(); // Scale normalized PID values for PWM output

  if (Flight_State::armed) {
    if(Flight_State::channel_pwm[5] < 1900){
      Hardware::command_hardware();
    } else {
      Hardware::esc.writeMicroseconds(Flight_State::channel_pwm[2]);
      Hardware::servo1.writeMicroseconds(Flight_State::channel_pwm[1]);
      Hardware::servo2.writeMicroseconds(Flight_State::channel_pwm[0]);
      Hardware::servo3.writeMicroseconds(Flight_State::channel_pwm[3]);
    }
  }
  // Hardware::esc.writeMicroseconds(Flight_State::channel_pwm[2]);
  // Hardware::servo1.writeMicroseconds(Flight_State::channel_pwm[0]);
  // Hardware::servo2.writeMicroseconds(Flight_State::channel_pwm[1]);

  // Get receiver commands in preparation for next iteration
  Receiver_Manager::update();
  if(Receiver_Manager::isFailsafe()) {
    Flight_State::failsafe_values(); //Write Failsafe values
  }
  get_commands();

  // print_receiver_input();
  print_IMU_data();       // Prints IMU data stored in Flight_State to serial
  print_attitude();
  // print_PID_values();
  print_scaled_outputs(); // Print the scaled outputs from the PID controller (what will be writted to ESC and Servos)
  

  // Call this function last in the loop to regulate loop frequency
  Timing::loop_rate(); // all filter parameters tuned to 2000Hz by default
}


void get_commands() {
  if (!Receiver_Manager::isReady()) return;

  if (Receiver_Manager::isFailsafe()) {
      Serial.println(" Failsafe is active");
      return;
  }

  uint16_t raw_inputs[Config::num_channels];
  for (int i = 0; i < Config::num_channels; ++i) {
      raw_inputs[i] = Receiver_Manager::getChannelUs(i);
  }

  Flight_State::update_from_raw_input(raw_inputs);
  Flight_State::get_des_values();
}

void print_receiver_input() {
  Serial.print("Receiver Inputs: ");
  Serial.print(" Ail:"); Serial.print(Flight_State::channel_pwm[0]);
  Serial.print(" Ele:"); Serial.print(Flight_State::channel_pwm[1]);
  Serial.print(" Thr:"); Serial.print(Flight_State::channel_pwm[2]);
  Serial.print(" Rud:"); Serial.print(Flight_State::channel_pwm[3]);
  Serial.print(" ARM:"); Serial.print(Flight_State::channel_pwm[4]);
  Serial.print(" IDK:"); Serial.println(Flight_State::channel_pwm[5]);
}

void get_IMU_data() {
  // Updates variables in Flight_State based on available IMU values
  // Call this function right after calling IMU_Manager::update()
  Flight_State::AccX = IMU_Manager::getAccelX();
  Flight_State::AccY = IMU_Manager::getAccelY();
  Flight_State::AccZ = IMU_Manager::getAccelZ();
  Flight_State::GyroX = IMU_Manager::getGyroX();
  Flight_State::GyroY = IMU_Manager::getGyroY();
  Flight_State::GyroZ = IMU_Manager::getGyroZ(); 
}

void print_IMU_data() {
  Serial.print("IMU data: ");
  Serial.print(" Ax:"); 
  if(Flight_State::AccX < 0) {
    Serial.print(Flight_State::AccX);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::AccX);
  }
  Serial.print(" Ay:"); 
    if(Flight_State::AccY < 0) {
    Serial.print(Flight_State::AccY);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::AccY);
  }
  Serial.print(" Az:");
    if(Flight_State::AccZ < 0) {
    Serial.print(Flight_State::AccZ);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::AccZ);
  }
  Serial.print(" Gx:");   if(Flight_State::GyroX < 0) {
    Serial.print(Flight_State::GyroX);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::GyroX);
  }
  Serial.print(" Gy:");   if(Flight_State::GyroY < 0) {
    Serial.print(Flight_State::GyroY);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::GyroY);
  }
  Serial.print(" Gz:");   if(Flight_State::GyroZ < 0) {
    Serial.print(Flight_State::GyroZ);
  } else {
    Serial.print("+");
    Serial.print(Flight_State::GyroZ);
  }
  Serial.println();
}

void print_attitude() {
  Serial.print("Attitude: ");
  Serial.print(" Roll: "); Serial.print(Flight_State::roll_IMU);
  Serial.print(" Pitch: "); Serial.print(Flight_State::pitch_IMU);
  Serial.print(" Yaw: "); Serial.println(Flight_State::yaw_IMU);
}

void calibrate_attitude() {
  //DESCRIPTION: Used to warm up the main loop to allow the madwick filter to converge before commands can be sent to the actuators
  //Assuming vehicle is powered up on level surface!
  /*
  * This function is used on startup to warm up the attitude estimation and is what causes startup to take a few seconds
  * to boot. 
  */
    //Warm up IMU and madgwick filter in simulated main loop
    for (int i = 0; i <= 10000; i++) {
        Timing::update();
        IMU_Manager::update();
        get_IMU_data();
        Flight_State::Madgwick(Flight_State::GyroX, -Flight_State::GyroY, -Flight_State::GyroZ, -Flight_State::AccX, Flight_State::AccY, Flight_State::AccZ, Flight_State::MagY, -Flight_State::MagX, Flight_State::MagZ, Timing::dt); //Updates roll_IMU, pitch_IMU, and yaw_IMU angle estimates (degrees)
        Timing::loop_rate(); //do not exceed 2000Hz
    }
  }

void print_scaled_outputs() {
  Serial.print("Scaled outputs: ");
  Serial.print(" Ele:"); Serial.print(PID::s1_command_scaled);
  Serial.print(" Ail:"); Serial.print(PID::s2_command_scaled);
  Serial.print(" Thr:"); Serial.print(PID::m1_command_scaled);
  Serial.print(" Rud:"); Serial.println(PID::s3_command_scaled);
  // Serial.print(" ARM:"); Serial.print(PID::s5_command_scaled);
  
}

void print_PID_values() {
  // This function is useful for PID parameter tuning.
  // You should be able to get PID values to range from -1 to 1 (or close to it)
  Serial.print("PID values: ");
  Serial.print(" roll_PID: "); Serial.print(PID::roll_PID);
  Serial.print(" pitch_PID: "); Serial.print(PID::pitch_PID);
  Serial.print(" yaw_PID: "); Serial.println(PID::yaw_PID);
}







