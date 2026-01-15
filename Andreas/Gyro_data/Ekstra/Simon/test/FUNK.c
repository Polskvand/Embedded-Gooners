#include "FUNK.h"
#include <math.h>
#include <stdio.h>


void init_acc_pos(acc_pos *gyro){
    gyro->ax = 0; gyro->ay = 0; gyro->az = 0;
    gyro->vx = 0; gyro->vy = 0; gyro->vz = 0;
    gyro->px = 0; gyro->py = 0; gyro->pz = 0;
    gyro->dt = 0.0001;
}

void step(float *gyro_axis_pos, float *gyro_axis_acc, float *gyro_axis_vel, float gyro_dt){
    *gyro_axis_vel += *gyro_axis_acc * gyro_dt;
    *gyro_axis_pos += *gyro_axis_vel * gyro_dt;
}



// void shift(float *gyro_axis, float new_acceleration){
//     // gyro_axis[0] = gyro_axis[1];
//     // gyro_axis[1] = new_acceleration;
// }

#define PI 3.141592653589793238f

void acc_sim(float t, float *ax, float *ay, float *az){
    // Rystelse parametre
    float Ax = 4.20f, Ay = 0.67f, Az = 0.69f;
    // Frequencer
    float fx = 2.23f, fy = 1.95f, fz = 5.52f;

    // Beregn accelerationer
    *ax = Ax * sinf(2.0f * PI * fx * t);
    *ay = Ay * sinf(2.0f * PI * fy * t);
    *az = Az * sinf(2.0f * PI * fz * t);
}