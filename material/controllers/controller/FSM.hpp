#pragma once

// Implemented by: Alice Bocquet, 379741

#include "pioneer_interface/pioneer_interface.hpp"
#include "braitenberg.hpp"

// =====================
// FSM STATES
// =====================

enum basicBehaviors {
  GOFORWARD,
  CLEAR_END,
  ARC_TURN,
  ENTER_ALLEY,
  STOP
};

static basicBehaviors behavior = GOFORWARD; // initialization

// =====================
// GLOBAL STATE
// =====================

// Count the number of turns to determine the turn direction
// 0 = alley 1 -> 2 : left
// 1 = alley 2 -> 3 : right
// 2 = alley 3 -> 4 : left
static int turn_index = 0;

// 1 ==> left turn, -1 ==> right turn
static int turn_direction = 1;

static bool encoders_initialized = false;

static double previous_left_encoder = 0.0;
static double previous_right_encoder = 0.0;

// Distance traveled in the current alley
static double distance_in_alley = 0.0;

// Distance traveled while clearing the end of the alley
static double clear_end_distance = 0.0;

// Angle of the arc turn
static double turn_angle = 0.0;

// Distance traveled while entering the next alley
static double enter_distance = 0.0;

// =====================
// PARAMETER DEFINITIONS
// =====================

// Pioneer dimensions
#define WHEEL_RADIUS 0.11
#define AXIS_LENGTH 0.40

// Map characteristics
#define N_ALLEYS 4

// Forward navigation
#define MAX_CORRECTION_NORMAL 0.50
#define MAX_CORRECTION_END 0.18

#define GOFORWARD_SPEED 0.49
#define ENTER_SPEED 0.32

// Start stabilization
// At the beginning of an alley, the robot is not perfectly aligned yet.
// The correction is reduced to avoid turning aggressively into the first pots.
#define START_STABILIZE_DISTANCE 0.60
#define MAX_CORRECTION_START 0.14
#define START_SPEED 0.42

// End of alley detection
#define ALLEY_DISTANCE 5.10
#define END_DETECTION_MIN_DISTANCE 4.65
#define END_STABILIZE_DISTANCE 4.50
#define FRONT_WALL_THRESHOLD 1000.0

// Clear end before turn
#define CLEAR_END_DISTANCE 0.42

// Arc turn
// The arc is mainly fixed to make all turns more uniform.
// The inner sensor is only used as a safety margin when the robot is too close to the pot.
#define ARC_TURN_ANGLE 4.10

#define ARC_INNER_SPEED 0.08
#define ARC_OUTER_SPEED 0.55

#define INNER_TOO_CLOSE 850.0
#define ARC_OPENING_GAIN 0.0008
#define ARC_MAX_OPENING 0.10

// Enter next alley
#define ENTER_DISTANCE 0.38

// =====================
// BASIC UTILS
// =====================

// Clamp a value between a min and a max
inline double clamp(double value, double min_value, double max_value){
  if(value < min_value) return min_value;
  if(value > max_value) return max_value;
  return value;
}

// Determine the maximum value among the front-left sensors (1, 2, 3)
inline double maxFrontLeft(double* ps_values){
  double value = ps_values[1];

  if(ps_values[2] > value) value = ps_values[2];
  if(ps_values[3] > value) value = ps_values[3];

  return value;
}

// Determine the maximum value among the front-right sensors (4, 5, 6)
inline double maxFrontRight(double* ps_values){
  double value = ps_values[4];

  if(ps_values[5] > value) value = ps_values[5];
  if(ps_values[6] > value) value = ps_values[6];

  return value;
}

// Determine the maximum value among the left sensors (0, 1, 2, 15)
inline double maxLeft(double* ps_values){
  double value = ps_values[0];

  if(ps_values[1] > value) value = ps_values[1];
  if(ps_values[2] > value) value = ps_values[2];
  if(ps_values[15] > value) value = ps_values[15];

  return value;
}

// Determine the maximum value among the right sensors (5, 6, 7, 8)
inline double maxRight(double* ps_values){
  double value = ps_values[5];

  if(ps_values[6] > value) value = ps_values[6];
  if(ps_values[7] > value) value = ps_values[7];
  if(ps_values[8] > value) value = ps_values[8];

  return value;
}

// Determine the turn direction based on the turn index
inline void chooseTurnDirection(){

  if(turn_index == 0){
    turn_direction = 1;      // left
  }
  else if(turn_index == 1){
    turn_direction = -1;     // right
  }
  else if(turn_index == 2){
    turn_direction = 1;      // left
  }
  else{
    turn_direction = 1;
  }
}

// Get the value of the inner sensor based on the turn direction
inline double getInnerSensor(double* ps_values){

  if(turn_direction == 1){
    return maxLeft(ps_values);
  }
  else{
    return maxRight(ps_values);
  }
}

// =====================
// ENCODER UTILS
// =====================

// Initialize the encoder reference values to the current wheel rotations
inline void resetEncoderReference(double* wheel_rot){

  previous_left_encoder = wheel_rot[0];
  previous_right_encoder = wheel_rot[1];

  encoders_initialized = true;
}

// Compute the distance traveled by each wheel since the last reference reset
inline void getEncoderDelta(double* wheel_rot, double &dist_left, double &dist_right){

  if(!encoders_initialized){
    resetEncoderReference(wheel_rot);

    dist_left = 0.0;
    dist_right = 0.0;

    return;
  }

  double left_encoder = wheel_rot[0];
  double right_encoder = wheel_rot[1];

  double delta_left = left_encoder - previous_left_encoder;
  double delta_right = right_encoder - previous_right_encoder;

  previous_left_encoder = left_encoder;
  previous_right_encoder = right_encoder;

  dist_left = WHEEL_RADIUS * delta_left;
  dist_right = WHEEL_RADIUS * delta_right;
}

// Update a traveled distance by averaging the distances from both wheels
inline void updateDistance(double* wheel_rot, double &distance){

  double dist_left, dist_right;
  getEncoderDelta(wheel_rot, dist_left, dist_right);

  double delta_distance = 0.5 * (dist_left + dist_right);

  if(delta_distance > 0.0){
    distance += delta_distance;
  }
}

// Update the turn angle by integrating the difference in distance traveled by the two wheels
inline void updateTurnAngle(double* wheel_rot){

  double dist_left, dist_right;
  getEncoderDelta(wheel_rot, dist_left, dist_right);

  double delta_theta = (dist_right - dist_left) / AXIS_LENGTH;

  turn_angle += delta_theta;
}

// =====================
// BEHAVIORS
// =====================

// GOFORWARD uses the Braitenberg controller to navigate forward while avoiding obstacles and keeping centered in the alley
inline void goForwardBehavior(double* ps_values, double &vel_left, double &vel_right){

  bool near_start = distance_in_alley < START_STABILIZE_DISTANCE;
  bool near_end = distance_in_alley > END_STABILIZE_DISTANCE;

  /*
   * Uniform rear-wheel protection:
   *
   * Rear straightening is enabled only in the middle of each alley.
   * It is disabled at the beginning because the robot still needs to align after
   * a turn, and disabled near the end because the robot must prepare for the arc turn.
   *
   * This rule is applied identically to all alleys.
   */
  bool allow_rear_straight = (!near_start && !near_end);

  if(near_start){
    /*
     * At the beginning of an alley, the robot is not perfectly aligned yet.
     * The correction is kept small to avoid an aggressive turn into the first pots.
     * Rear straightening is disabled here to allow proper alignment.
     */
    braitenberg(
      ps_values,
      vel_left,
      vel_right,
      true,
      MAX_CORRECTION_START,
      START_SPEED,
      false
    );
  }
  else if(near_end){
    /*
     * Near the end of the alley, the correction is reduced to avoid oscillations
     * before starting to turn. Rear straightening is disabled here.
     */
    braitenberg(
      ps_values,
      vel_left,
      vel_right,
      false,
      MAX_CORRECTION_END,
      GOFORWARD_SPEED,
      false
    );
  }
  else{
    /*
     * Middle of the alley: normal Braitenberg navigation with rear protection.
     */
    braitenberg(
      ps_values,
      vel_left,
      vel_right,
      true,
      MAX_CORRECTION_NORMAL,
      GOFORWARD_SPEED,
      allow_rear_straight
    );
  }
}

// endOfAlleyDetected checks if the robot has reached the end of the alley based on the distance traveled and the readings of the front sensors
inline bool endOfAlleyDetected(double* ps_values){

  double front_left = maxFrontLeft(ps_values);
  double front_right = maxFrontRight(ps_values);

  bool enough_distance = distance_in_alley > END_DETECTION_MIN_DISTANCE;
  bool distance_reached = distance_in_alley > ALLEY_DISTANCE;

  bool front_wall =
    front_left > FRONT_WALL_THRESHOLD &&
    front_right > FRONT_WALL_THRESHOLD;

  return enough_distance && (distance_reached || front_wall);
}

// clearEndBehavior moves the robot forward to clear the end of the alley before starting the turn
inline void clearEndBehavior(double &vel_left, double &vel_right){

  vel_left = 0.22;
  vel_right = 0.22;
}

// arcTurnBehavior performs a mostly fixed arc turn.
// The proximity sensor is only used to open the arc if the robot gets too close to the inner pot.
inline void arcTurnBehavior(double* ps_values, double &vel_left, double &vel_right){

  double inner = getInnerSensor(ps_values);

  /*
   * Robust arc strategy:
   *
   * The turn is mainly controlled by fixed wheel speeds and encoder angle.
   * This makes the behaviour similar for all turns, even if the spacing between
   * pots changes slightly.
   *
   * The inner proximity sensor is only used as a safety margin:
   * if the robot gets too close to the pot around which it turns, the arc is opened.
   *
   * Important:
   * If the inner sensor value is low, we do not tighten the arc. A low value may
   * simply mean that the end of the alley is wider, not that the robot should turn
   * more aggressively.
   */

  double inner_speed = ARC_INNER_SPEED;
  double outer_speed = ARC_OUTER_SPEED;

  if(inner > INNER_TOO_CLOSE){
    // Too close to the inner pot: open the arc
    double opening = ARC_OPENING_GAIN * (inner - INNER_TOO_CLOSE);
    opening = clamp(opening, 0.0, ARC_MAX_OPENING);

    inner_speed += opening;
    outer_speed -= opening;
  }

  if(turn_direction == 1){

    // Left arc turn
    vel_left = inner_speed;
    vel_right = outer_speed;
  }
  else{

    // Right arc turn
    vel_left = outer_speed;
    vel_right = inner_speed;
  }

  vel_left = clamp(vel_left, 0.08, 0.55);
  vel_right = clamp(vel_right, 0.08, 0.55);
}

// enterAlleyBehavior moves the robot forward to enter the next alley after completing the turn
inline void enterAlleyBehavior(double* ps_values, double &vel_left, double &vel_right){

  /*
   * The robot also uses Braitenberg while entering the next alley.
   * This short recentering phase helps the robot recover after the turn
   * before starting the normal forward navigation.
   *
   * Rear straightening is disabled here because the robot still needs to turn
   * and align with the new alley.
   */

  braitenberg(ps_values, vel_left, vel_right, true, 0.22, ENTER_SPEED, false);
}

// stopBehavior stops the robot by setting both wheel velocities to zero
inline void stopBehavior(double &vel_left, double &vel_right){

  vel_left = 0.0;
  vel_right = 0.0;
}

// =====================
// MAIN FSM
// =====================

inline void fsm(double* ps_values, double* wheel_rot, double time, double &vel_left, double &vel_right){

  (void)time; // time is not used in this FSM

  switch(behavior){

    case GOFORWARD: {

      updateDistance(wheel_rot, distance_in_alley);
      goForwardBehavior(ps_values, vel_left, vel_right);

      if(endOfAlleyDetected(ps_values)){

        if(turn_index >= N_ALLEYS - 1){
          behavior = STOP;
        }
        else{
          chooseTurnDirection();

          behavior = CLEAR_END;
          clear_end_distance = 0.0;

          resetEncoderReference(wheel_rot);
        }
      }

      break;
    }

    case CLEAR_END: {

      updateDistance(wheel_rot, clear_end_distance);
      clearEndBehavior(vel_left, vel_right);

      if(clear_end_distance > CLEAR_END_DISTANCE){

        behavior = ARC_TURN;
        turn_angle = 0.0;

        resetEncoderReference(wheel_rot);
      }

      break;
    }

    case ARC_TURN: {

      updateTurnAngle(wheel_rot);
      arcTurnBehavior(ps_values, vel_left, vel_right);

      double signed_angle = turn_direction * turn_angle;

      if(signed_angle > ARC_TURN_ANGLE){

        turn_index++;

        behavior = ENTER_ALLEY;
        enter_distance = 0.0;

        resetEncoderReference(wheel_rot);
      }

      break;
    }

    case ENTER_ALLEY: {

      updateDistance(wheel_rot, enter_distance);
      enterAlleyBehavior(ps_values, vel_left, vel_right);

      if(enter_distance > ENTER_DISTANCE){

        behavior = GOFORWARD;
        distance_in_alley = 0.0;

        resetEncoderReference(wheel_rot);
      }

      break;
    }

    case STOP: {

      stopBehavior(vel_left, vel_right);
      break;
    }

    default: {

      stopBehavior(vel_left, vel_right);
      behavior = STOP;

      break;
    }
  }
}
