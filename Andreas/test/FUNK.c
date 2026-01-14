#include "FUNK.h"
#include <math.h>
#include <stdio.h>


void init_acc_pos(acc_pos *gyro){
    for(int i = 0; i <= 1; i++){
        // Initialisere alle accelerationer til 0: 
        // Todo: Sæt z til 1 hvis det er op
        gyro->ax[i] = 0; gyro->ay[i] = 0; gyro->az[i] = 0;

        //Initialisere alle positioner til 0:
        gyro->px[i] = 0; gyro->py[i] = 0; gyro->pz[i] = 0;
    }
    gyro->dt = 0.0001;
}

static inline float step(float p2, float p1, float a2, float a1, float a0){
    
}

static inline float step_pos(float p2, float p1, float a2, float a1, float a0, float dt)
{
    float c = 0.25f * dt * dt; // Δt^2/4
    return 2.0f*p1 - p2 + c*(a2 + 2.0f*a1 + a0);
}


void shift(acc_pos *gyro){
    printf("%03.f", gyro->dt);

}


// static inline float step(float p2, float p1, float a2, float a1, float a0, float dt)
// {
//     float c = 0.25f * dt * dt; // Δt^2/4
//     return 2.0f*p1 - p2 + c*(a2 + 2.0f*a1 + a0);
// }

// void update_step(acc_pos *s, float ax_new, float ay_new, float az_new)
// {
//     // compute new positions p_k
//     float px_new = step_pos(s->px2, s->px1, s->ax2, s->ax1, ax_new, s->dt);
//     float py_new = step_pos(s->py2, s->py1, s->ay2, s->ay1, ay_new, s->dt);
//     float pz_new = step_pos(s->pz2, s->pz1, s->az2, s->az1, az_new, s->dt);

//     // shift position history
//     s->px2 = s->px1; s->px1 = px_new;
//     s->py2 = s->py1; s->py1 = py_new;
//     s->pz2 = s->pz1; s->pz1 = pz_new;

//     // shift accel history
//     s->ax2 = s->ax1; s->ax1 = ax_new;
//     s->ay2 = s->ay1; s->ay1 = ay_new;
//     s->az2 = s->az1; s->az1 = az_new;
// }