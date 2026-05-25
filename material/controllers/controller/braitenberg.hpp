#pragma once

// Implemented by: Alice Bocquet, 379741

#include "pioneer_interface/pioneer_interface.hpp"
#include <algorithm>

/*

Pioneer sensor layout:

                Front
                3   4
        1   2           5   6
    0                           7
Left                             Right
    15                          8
        14  13          10  9
                12  11
                 Back

Response: >5m -> 0, 0m -> 1024
*/

/**
 * @brief      This function implements the Braitenberg algorithm
 *              to control the robot's velocity.
 * @param      ps           Proximity sensor readings (NUM_SENSORS values)
 * @param      vel_left     The left velocity
 * @param      vel_right    The right velocity
*/

// The Braitenberg function uses the proximity sensor values to compute the left
// and right wheel velocities in order to keep the robot moving while avoiding side and front obstacles.

double max3(double a, double b, double c){
    return std::max(std::max(a, b), c);
}

double max4(double a, double b, double c, double d){
    return std::max(std::max(a, b), std::max(c, d));
}

void braitenberg(
    double* ps,
    double &vel_left,
    double &vel_right,
    bool use_side_correction = true,
    double max_correction = 0.50,
    double base_speed = 0.49,
    bool allow_rear_straight = true
){

    const double FRONT_GAIN = 0.010;         // strength of the front side correction
    const double SIDE_GAIN = 0.004;          // strength of the side obstacle correction
    const double DEAD_BAND = 30.0;           // ignore small sensor differences

    /*
     * Rear-side emergency straightening.
     * If the side/rear side is very close to a pot, but the front is not blocked,
     * the robot stops steering and moves straight forward.
     * This avoids rear-wheel sweeping into the obstacle.
     *
     * This mode is controlled by allow_rear_straight.
     * In the FSM, it is only allowed in the first alley because it helps avoid
     * the initial rear-wheel contact, but it disturbed the robot in the next alleys.
     */
    const double SIDE_CRITICAL = 900.0;
    const double REAR_CRITICAL = 900.0;
    const double FRONT_BLOCKED = 980.0;
    const double STRAIGHT_SPEED = 0.42;

    // Saturation to keep reasonable speeds
    const double MIN_SPEED = 0.00;
    const double MAX_SPEED = 0.70;

    // front obstacle detection
    double front_left = max3(ps[1], ps[2], ps[3]);
    double front_right = max3(ps[4], ps[5], ps[6]);
    double front = std::max(front_left, front_right);

    // left obstacle detection
    double left = max4(ps[0], ps[1], ps[2], ps[15]);

    // right obstacle detection
    double right = max4(ps[5], ps[6], ps[7], ps[8]);

    // rear-side obstacle detection
    double rear_left = max3(ps[13], ps[14], ps[15]);
    double rear_right = max3(ps[8], ps[9], ps[10]);

    /*
     * Emergency straight mode:
     * If the robot is grazing a pot on the side or rear-side, and the front is
     * not completely blocked, the safest action is to stop turning for this step.
     *
     * This is stronger than setting correction = 0 later, because we directly
     * set equal wheel velocities and return.
     */
    if(use_side_correction && allow_rear_straight && front < FRONT_BLOCKED){

        bool left_critical =
            left > SIDE_CRITICAL ||
            rear_left > REAR_CRITICAL;

        bool right_critical =
            right > SIDE_CRITICAL ||
            rear_right > REAR_CRITICAL;

        if(left_critical || right_critical){
            vel_left = STRAIGHT_SPEED;
            vel_right = STRAIGHT_SPEED;
            return;
        }
    }

    // Calculate the correction for the direction
    double diff_front = front_right - front_left;
    double diff_side = right - left;

    double correction = 0.0;

    // Correction based on front-left and front-right sensors
    if(diff_front > DEAD_BAND){
        correction += FRONT_GAIN * (diff_front - DEAD_BAND);
    }
    else if(diff_front < -DEAD_BAND){
        correction += FRONT_GAIN * (diff_front + DEAD_BAND);
    }

    // Correction based on side sensors
    if(use_side_correction){
        if(diff_side > DEAD_BAND){
            correction += SIDE_GAIN * (diff_side - DEAD_BAND);
        }
        else if(diff_side < -DEAD_BAND){
            correction += SIDE_GAIN * (diff_side + DEAD_BAND);
        }
    }

    // Limit the turning correction to avoid brutal rotations
    correction = std::max(-max_correction, std::min(max_correction, correction));

    // Compute wheel speeds
    vel_left = base_speed - correction;
    vel_right = base_speed + correction;

    // Saturation to keep reasonable speeds
    vel_left = std::max(MIN_SPEED, std::min(MAX_SPEED, vel_left));
    vel_right = std::max(MIN_SPEED, std::min(MAX_SPEED, vel_right));
}
