#pragma once #pragma once 

// Implemented by: Alice Bocquet, 379741

#include "pioneer_interface/pioneer_interface.hpp"

#define PACKET_SIZE 5 // Number of doubles in a packet 



/**
 * @brief      Print an array of doubles in the terminal
 * @param[in]  array The array
*/
void print_array(double* array){
    printf("[");
    for(int i=0; i<PACKET_SIZE; i++){
        printf("%.3lf, ", array[i]);
    }
    printf("]\n");
}

/**
 * @brief      Get a message from the serial port
 * @param[in]  robot The robot object
 * @param[out] data  The data destination 
 * @return     signal strength of the received packet, 0.0 if no packet was received
*/
double serial_get_data(Pioneer& robot, double* data){

    // If no packet is available, return 0.0
    if(robot.serial_get_queue_length() == 0){
        return 0.0;
    }

    // Read the current packet
    const double* msg = robot.serial_read_msg();

    // Copy the packet values into the output array
    for(int i=0; i<PACKET_SIZE; i++){
        data[i] = msg[i];
    }

    // Get the signal strength of the received packet
    double signal_strength = robot.serial_get_signal_strength();

    // Move to the next packet in the queue
    robot.serial_next_msg();

    return signal_strength; 
}
