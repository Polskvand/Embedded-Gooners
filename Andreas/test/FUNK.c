#include "FUNK.h"
#include <math.h>
#include <stdio.h>


void init_acc_pos(acc_pos *gyro){
    for(int i = 0; i <= 1; i++){
        // Initialisere alle accelerationer til 0: 
        // Todo: Sæt z til 1 hvis det er op
        gyro->ax[i] = 0; gyro->ay[i] = 0; gyro->az[i] = 0;
    }
    //Initialisere alle positioner og velocity til 0:
    gyro->px = 0; gyro->py = 0; gyro->pz = 0;
    gyro->vx = 0; gyro->vy = 0; gyro->vz = 0;
    gyro->dt = 0.0001;
}


void shift(float *gyro_axis, float new_acceleration){
    gyro_axis[0] = gyro_axis[1];
    gyro_axis[1] = new_acceleration;
}

void step(const float *acc_axis, float *pos_axis, float *vel_axis, float dt){
    float v_old = *vel_axis;
    *vel_axis = v_old + 0.5f * dt * (acc_axis[0] + acc_axis[1]);
    *pos_axis = *pos_axis + dt * v_old + 0.25f * dt * dt * (acc_axis[0] + acc_axis[1]);
}

#define PI 3.14159265358979f
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

