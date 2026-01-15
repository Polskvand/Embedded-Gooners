#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct {
    // acceleration history: a_{k-2}, a_{k-1}
    float ax[2], ay[2], az[2];

    // velocity history: v_{k-1}
    float vx, vy, vz;

    // position history: p_{k-1}
    float px, py, pz;

    float dt; // sample time [s]
} acc_pos;

void init_acc_pos(acc_pos *gyro);
void shift(float *gyro_axis, float new_acceleration);
void step(const float *acc_axis, float *pos_axis, float *vel_axis, float dt);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif

