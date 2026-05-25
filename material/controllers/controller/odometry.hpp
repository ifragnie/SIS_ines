#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

inline double wrap_angle(double a){

    while(a > M_PI)  a -= 2.0 * M_PI;
    while(a < -M_PI) a += 2.0 * M_PI;

    return a;
}

//robot parameter 
void compute_odometry(double*wheel_rot, double dt, double*v, double*w, double* distL_out, double* distR_out){
    //Robot size
    const double R = PioneerInfo::wheel_radius;
    const double L = PioneerInfo::axis_length;

    if (dt < 1e-9) {
        *v = 0; *w = 0; *distL_out = 0; *distR_out = 0;
        return;
    }

    //Static to keep track of previous wheel rotations
    static double prev_left =0.0;
    static double prev_right=0.0;
    static bool first= true;

    //New wheel rotations
    double left_wheel = wheel_rot[0];
    double right_wheel = wheel_rot[1];

    // if it is the first time running ?
    if (first){
        prev_left=left_wheel;
        prev_right=right_wheel;
        first=false;
        *v=0; *w=0; *distL_out = 0; *distR_out = 0;
        return;
    }

    //change in wheel motion
    double dLeft=left_wheel - prev_left;
    double dRight=right_wheel - prev_right;

    *distL_out = dLeft  * R;
    *distR_out = dRight * R;

    printf("dL=%.6f  dR=%.6f  leftW=%.6f  rightW=%.6f\n", dLeft, dRight, left_wheel, right_wheel);

    //convert to linear dist
    double distL=dLeft*R;
    double distR=dRight*R;

    //velocities

    *v = (*distR_out + *distL_out) / (2.0 * dt);
    *w = (*distR_out - *distL_out) / (L   * dt);
    
    //update
    prev_left = left_wheel;
    prev_right = right_wheel;
}#pragma once 

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <memory.h>

#include "pioneer_interface/pioneer_interface.hpp"

#define RAD2DEG(X)      X / M_PI * 180.0 // Convert radians to degrees

inline double wrap_angle(double a){

    while(a > M_PI)  a -= 2.0 * M_PI;
    while(a < -M_PI) a += 2.0 * M_PI;

    return a;
}

//robot parameter 
void compute_odometry(double*wheel_rot, double dt, double*v, double*w, double* distL_out, double* distR_out){
    //Robot size
    const double R = PioneerInfo::wheel_radius;
    const double L = PioneerInfo::axis_length;

    if (dt < 1e-9) {
        *v = 0; *w = 0; *distL_out = 0; *distR_out = 0;
        return;
    }

    //Static to keep track of previous wheel rotations
    static double prev_left =0.0;
    static double prev_right=0.0;
    static bool first= true;

    //New wheel rotations
    double left_wheel = wheel_rot[0];
    double right_wheel = wheel_rot[1];

    // if it is the first time running ?
    if (first){
        prev_left=left_wheel;
        prev_right=right_wheel;
        first=false;
        *v=0; *w=0; *distL_out = 0; *distR_out = 0;
        return;
    }

    //change in wheel motion
    double dLeft=left_wheel - prev_left;
    double dRight=right_wheel - prev_right;

    *distL_out = dLeft  * R;
    *distR_out = dRight * R;

    //convert to linear dist
    double distL=dLeft*R;
    double distR=dRight*R;

    //velocities

    *v = (*distR_out + *distL_out) / (2.0 * dt);
    *w = (*distR_out - *distL_out) / (L   * dt);


    // TRUC QUI FONCTIONNE MAIS ESSAYE DE FAIRE MIEUX !!!
    /*
    *v=(distR+distL)/(2*dt);
    *w=(distR-distL)/(L*dt);

    double slip_factor = 1.0 - 0.05 * fabs(*w);
    *w *= 0.7;
    *v *= 1.0 / (1.0 + 0.85 * fabs(*w));
    printf("v=%.5f  w=%.5f\n", *v, *w);
*/



    //update
    prev_left = left_wheel;
    prev_right = right_wheel;
}
