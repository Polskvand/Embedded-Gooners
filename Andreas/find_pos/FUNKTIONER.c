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
    }
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
