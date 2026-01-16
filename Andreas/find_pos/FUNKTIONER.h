#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct{
    float p, v, a;
} Axis_state;

typedef struct{
    Axis_state x, y, z;
} gyro_state;

void init_gyro(gyro_state *gyro, float ax0, float ay0, float az0);
void step(Axis_state *axis, float acc_new);

#endif