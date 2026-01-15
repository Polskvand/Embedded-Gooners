#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct {
    // acceleration history: a_{k-2}, a_{k-1}
    float ax[2], ay[2], az[2]; 

    // position history: p_{k-2}, p_{k-1}
    float px, py, pz;

    float dt; // sample time [s]
} acc_pos;

void init_acc_pos(acc_pos *gyro);
void step(acc_pos *gyro);
void shift(float *gyro_axis, float new_acceleration);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif

