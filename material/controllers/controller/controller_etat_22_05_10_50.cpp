// Controller for the robot
// Implemented by: Lou Blassel, 379739
// Implemented by: Alice Bocquet, 379741
// Implemented by: Inès Fragnière, 363961
#include <stdio.h>
#include <string>


// Provided libraries 
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

// Files to implement your solutions  
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"


#include <vector>
#include <iostream>

#define SIGNAL_LENGTH 128 // Define the FFT signal length



// =======================
// FILTER STATE
// =======================

// parameters (à tuner)
static const double alpha = 0.1;        // smoothing
static const double threshold = 0.6;    // detection threshold
static const double cooldown = 1.0;     // avoid double detection (sec)

double filtered_light;




int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Initialize temperature log file for WP1
  std::string f_temp = "temperature.csv";
  int f_temp_cols = init_csv(
    f_temp,
    "time, sensor_id, node_x, node_y, T_in, T_out, signal_strength,"
  );

  while (robot.step() != -1) {
    //////////////////////////////
    // Measurements acquisition //
    //////////////////////////////
    
    double  time = robot.get_time();              // Current time in seconds 
    double* ps_values = robot.get_proximity();    // Measured proximity sensor values (16 values)
    double* wheel_rot = robot.get_encoders();     // Wheel rotations (left, right)
    double  light = robot.get_light_intensity();  // Light intensity
    double* imu = robot.get_imu();                // IMU with accelerations and rotation rates (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)

    
    
    ////////////////////
    // Implementation //
    ////////////////////

    // ----------------------------
    // TEMPERATURE DATA ACQUISITION
    // ----------------------------

    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // Log temperature data only when a packet is received
    if(signal_strength > 0.0){
      log_csv(
        f_temp,
        f_temp_cols,
        time,
        data[0],          // sensor_id
        data[1],          // node_x
        data[2],          // node_y
        data[3],          // T_in
        data[4],          // T_out
        signal_strength
      );
    }

    // ----------
    // NAVIGATION
    // ----------

    double lws = 0.0;
    double rws = 0.0;

    fsm(ps_values, wheel_rot, time, lws, rws);

    robot.set_motors_velocity(lws, rws);

    // STATE ESTIMATION MARKER
    robot.hide_state_estimate_marker();



    // NAVIGATION


    // STATE ESTIMATION MARKER (green arrow in simulation)

    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!

    //////////////////
    // Data logging //
    //////////////////


    filtered_light = light_analysis(time, light);

  // Add the light intensity value to the buffer
  //static int fft_count = 0;
  // Remplissage initial du buffer
  //if (buffer_index < SIGNAL_LENGTH) {
    //light_signal_buffer[buffer_index++] = light;
    //std::cout << "buffer fill: " << buffer_index << std::endl;
  //}
  //else {
    //for (int i = 0; i < SIGNAL_LENGTH - 1; i++) {
      //light_signal_buffer[i] = light_signal_buffer[i + 1];
    //}

    //light_signal_buffer[SIGNAL_LENGTH - 1] = light;

    //kiss_fft_demo(light_signal_buffer);
    //std::cout << "FFT #" << fft_count << std::endl;
  //fft_count++;
  //}

  // Enter here exit cleanup code.
  }
  close_csv(); // close all opened csv files
  return 0;
}
