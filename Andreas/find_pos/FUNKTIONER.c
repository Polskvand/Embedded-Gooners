#include "FUNKTIONER.h"
#include <stdio.h>
#include <math.h>

void init_gyro(gyro_state *gyro, float ax0, float ay0, float az0){
    *gyro = (gyro_state){0};
    gyro->x.a = ax0;
    gyro->y.a = ay0;
    gyro->z.a = az0;
}

void step(Axis_state *axis, float acc_new){
    float dt = 0.001;
    float vel_old = axis->v;
    axis->v = vel_old + 0.5f * dt * (acc_new + axis->a);
    axis->p = axis->p + dt * vel_old + 0.25f*dt*dt*(axis->a + acc_new);
    axis->a = acc_new;
}