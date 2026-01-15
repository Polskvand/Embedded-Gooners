#include "FUNKTIONER.h"
#include <math.h>

#define PI 3.14159265359f

void init_acc_pos(acc_pos *gyro){
    for(int i = 0; i <= 1; i++){
        // Initialisere alle accelerationer til 0: 
        // Todo: Men sæt z til 1 hvis det er op
        gyro->ax[i] = 0; gyro->ay[i] = 0; gyro->az[i] = 0;

        //Initialisere alle positioner til 0:
        gyro->px[i] = 0; gyro->py[i] = 0; gyro->pz[i] = 0;
        gyro->px[i+2] = 0; gyro->py[i+2] = 0; gyro->pz[i+2] = 0;
    }
}

void update_step(acc_pos *gyro, float ax_new, float ay_new, float az_new){
    float dt2_fjerde = 0.0001f;

    // Calculate next position
    gyro->px[3] = 2*(gyro->px[1]) - gyro->px[0] + dt2_fjerde*(ax_new + 2*gyro->ax[1] + gyro->ax[0]);
    gyro->py[3] = 2*(gyro->py[1]) - gyro->py[0] + dt2_fjerde*(ay_new + 2*gyro->ay[1] + gyro->ay[0]);
    gyro->pz[3] = 2*(gyro->pz[1]) - gyro->pz[0] + dt2_fjerde*(az_new + 2*gyro->az[1] + gyro->az[0]);

    // Increment all values
    gyro->px[0] = gyro->px[1]; gyro->px[1] = gyro->px[2]; gyro->px[2] = gyro->px[3];
    gyro->py[0] = gyro->py[1]; gyro->py[1] = gyro->py[2]; gyro->py[2] = gyro->py[3];
    gyro->pz[0] = gyro->pz[1]; gyro->pz[1] = gyro->pz[2]; gyro->pz[2] = gyro->pz[3];

    gyro->ax[0] = gyro->ax[1]; gyro->ax[1] = ax_new; 
    gyro->ay[0] = gyro->ay[1]; gyro->ay[1] = ay_new; 
    gyro->az[0] = gyro->az[1]; gyro->az[1] = az_new; 
}

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