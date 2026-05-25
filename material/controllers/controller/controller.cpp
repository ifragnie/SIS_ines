// Controller for the robot
// Implemented by: Lou Blassel, 379739
// Implemented by: Alice Bocquet, 379741
// Implemented by: Inès Fragnière, 363961

// Provided libraries 
#include <stdio.h>
#include <string>
#include <vector>
#include <iostream>

// Files to implement your solutions  
#include "braitenberg.hpp"
#include "odometry.hpp"
#include "kalman.hpp"
#include "FSM.hpp"
#include "serial.hpp"
#include "signal_analysis.hpp"
#include "pioneer_interface/pioneer_interface.hpp"
#include "utils/log_data.hpp"

int main(int argc, char **argv) {

  // Initialize the robot
  Pioneer robot = Pioneer(argc, argv);
  robot.init();
  
  /// Initialize temperature log file for WP1
  std::string f_temp = "temperature.csv";
  int f_temp_cols = init_csv(f_temp, "time, sensor_id, node_x, node_y, T_in, T_out, signal_strength,");

  // Initialize ekf estimation file for WP2
  std::string f_est = "ekf_estimation.csv";
  int f_est_cols = init_csv(f_est, "time, est_x, est_y, est_theta, sigma_x, sigma_y,sigma_theta,");

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

    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, wheel_rot, time, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities
    
    // STATE ESTIMATION MARKER (green arrow in simulation)
    robot.hide_state_estimate_marker(); // this hides the marker in the simulation
//    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!

    double dt = robot.get_timestep()/1000.0; // convert timestep to seconds

    double v = 0.0;
    double w = 0.0;
    
    double distL = 0.0, distR = 0.0;
    double gyro_z = imu[5];
    compute_odometry(wheel_rot, dt, &v, &w, &distL, &distR);

    double cov[3][3];
    double* cov_rows[3] = { cov[0], cov[1], cov[2] };
    double state[3];
    kal_get_state(state);
    
    if (signal_strength >= 1.9) {
        kal_prediction(distL, distR, gyro_z, dt, true);
        kal_update_heading_axis();
        double dx = data[1] - state[0];
        double dy = data[2] - state[1];
        double dist_to_node = sqrt(dx*dx + dy*dy);
        kal_update_position(data[1],data[2],dist_to_node);
    } else {
        kal_prediction(distL, distR, gyro_z, dt, false);
    }
    
    kal_get_state(state);
    kal_get_state_covariance(cov_rows);
    
    robot.set_state_estimate_marker(state[0],state[1], state[2]);
    
    //////////////////
    // Data logging //
    //////////////////        
        
    log_csv(f_est,
          f_est_cols,
          time,
          state[0],
          state[1],
          state[2],
          cov[0][0],
          cov[1][1],
          cov[2][2]);

    ////////////
    // Lights //
    ////////////

    light_analysis(robot, state[0], state[1]);

  }

  close_csv();

  return 0;
}// Controller for the robot
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

int main(int argc, char **argv) {

  // Initialize the robot
  Pioneer robot = Pioneer(argc, argv);
  robot.init();
  
  // Initialize an example log file
  // std::string f_example = "example.csv";
  // int         f_example_cols = init_csv(f_example, "time, light, accx, accy, accz,"); // <-- don't forget the comma at the end of the string!!

  // Initialize temperature log file for WP1
  std::string f_temp = "temperature.csv";
  int f_temp_cols = init_csv(f_temp, "time, sensor_id, node_x, node_y, T_in, T_out, signal_strength,");

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

//      printf("TEMP PACKET | t=%.2f sensor=%.0f x=%.2f y=%.2f T_in=%.2f T_out=%.2f signal=%.3f\n",
//        time,
//        data[0],
//        data[1],
//        data[2],
//        data[3],
//        data[4],
//        signal_strength);
    }

    // ----------
    // NAVIGATION
    // ----------

    double lws = 0.0, rws = 0.0;  // left and right wheel speeds
    fsm(ps_values, wheel_rot, time, lws, rws);     // finite state machine 
    robot.set_motors_velocity(lws, rws); // set the wheel velocities
    
    // STATE ESTIMATION MARKER (green arrow in simulation)
    robot.hide_state_estimate_marker(); // this hides the marker in the simulation
//    robot.set_state_estimate_marker(0.0, 0.0, time, time); // rotate in place for now, input your state estimate here for visualization in the simulation!


    
    double dt = robot.get_timestep()/1000.0; // convert timestep to seconds
    printf("dt=%.6f\n", dt);
   
    double v = 0.0;
    double w = 0.0;
    
    /*TRUC QUI MARCHE MAIS ON TESTE UN NOUVEAU
    compute_odometry(wheel_rot, dt, &v,&w);
    
    
    //Kalman update
    if(signal_strength >= 1.9 ) //si proche du node   
    {
      kal_prediction(v,w,dt,true);
      kal_update_position(data[1], data[2], 1/sqrt(signal_strength));   
    } else // Si loin du node
    { 
      kal_prediction(v,w,dt, false);
    }
  */
    
    double distL = 0.0, distR = 0.0;
    double gyro_z = imu[5];
    compute_odometry(wheel_rot, dt, &v, &w, &distL, &distR);

    double cov[3][3];
    double* cov_rows[3] = { cov[0], cov[1], cov[2] };
    double state[3];
    kal_get_state(state);
    
    if (signal_strength >= 1.9) {
        //double alpha = 1.0 / (1.0 + exp(-10.0 * (signal_strength - 1.98)));
        kal_prediction(distL, distR, gyro_z, dt, true);
        kal_update_heading_axis();
        double dx = data[1] - state[0];
        double dy = data[2] - state[1];
        double dist_to_node = sqrt(dx*dx + dy*dy);
        kal_update_position(data[1],data[2],dist_to_node);
        //kal_update_heading_axis();
    } else {
        kal_prediction(distL, distR, gyro_z, dt, false);
    }
    
    
    // double cov[3][3];
    // double* cov_rows[3] = { cov[0], cov[1], cov[2] };
    // double state[3];
    kal_get_state(state);
    kal_get_state_covariance(cov_rows);
    
    robot.set_state_estimate_marker(state[0],state[1], state[2]);
    
    //////////////////
    // Data logging //
    //////////////////

    printf("x=%f y=%f theta=%f\n",
        state[0],
        state[1],
        state[2]);
        
        
    log_csv(f_est,
          f_est_cols,
          time,
          state[0],
          state[1],
          state[2],
          cov[0][0],
          cov[1][1],
          cov[2][2]);
    // ------------
    // Data logging
    // ------------

    log_csv(
      f_example,
      f_example_cols,
      time,
      light,
      imu[0],
      imu[1],
      imu[2]
    );

    ////////////
    // Lights //
    ////////////

    light_analysis(robot, state[0], state[1]);

  }

  close_csv();

  return 0;
}
