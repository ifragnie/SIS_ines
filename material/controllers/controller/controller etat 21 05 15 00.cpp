// Controller for the robot robot

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

//#define SIGNAL_LENGTH 300 // Define the FFT signal length


// Buffer to store light signal values
//double light_signal_buffer[SIGNAL_LENGTH] = {0}; // Initialize buffer with zeros
//int buffer_index = 0; // Initialize buffer index


// =======================
// FILTER STATE
// =======================
static double filtered_light = 0.0;
static double baseline = 0.0;

// detection state
static bool in_light = false;
static double last_event_time = -1.0;

// parameters (à tuner)
static const double alpha = 0.1;        // smoothing
static const double threshold = 0.6;    // detection threshold
static const double cooldown = 1.0;     // avoid double detection (sec)




int main(int argc, char **argv) {

  // Initialize the robot 
  Pioneer robot = Pioneer(argc, argv);
  robot.init();

  // Initialize an example log file
  std::string f_example = "example.csv";
  int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!



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

    // DATA ACQUISITION
    double data[PACKET_SIZE];
    double signal_strength = serial_get_data(robot, data);

    // NAVIGATION
    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities

    // STATE ESTIMATION MARKER (green arrow in simulation)
    robot.hide_state_estimate_marker(); // this hides the marker in the simulation
    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!

    //////////////////
    // Data logging //
    //////////////////
        // Add the light intensity value to the buffer
static int fft_count = 0;
// Remplissage initial du buffer
if (buffer_index < SIGNAL_LENGTH) {
  light_signal_buffer[buffer_index++] = light;
  std::cout << "buffer fill: " << buffer_index << std::endl;
}
else {
  for (int i = 0; i < SIGNAL_LENGTH - 1; i++) {
    light_signal_buffer[i] = light_signal_buffer[i + 1];
  }

  light_signal_buffer[SIGNAL_LENGTH - 1] = light;

  kiss_fft_demo(light_signal_buffer);
  std::cout << "FFT #" << fft_count << std::endl;
fft_count++;
}



        // Logging
        log_csv(f_example, f_example_cols, time, light);

        printf("Light intensity: %.6f\n", light);
        (void)wheel_rot;
}
  // Enter here exit cleanup code.
  close_csv(); // close all opened csv files

  return 0;
}
