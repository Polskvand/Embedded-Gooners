#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct {
    // acceleration history: a_{k-2}, a_{k-1}
    float ax, ay, az;
    
    //hastighed
    float vx, vy, vz; 

    // position history: p_{k-2}, p_{k-1}
    float px, py, pz;

    float dt; // sample time [s]
} acc_pos;

void init_acc_pos(acc_pos *gyro);
void step(float *gyro_axis_pos, float *gyro_axis_acc, float *gyro_axis_vel, float gyro_dt);
// void shift(float *gyro_axis, float new_acceleration);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif

