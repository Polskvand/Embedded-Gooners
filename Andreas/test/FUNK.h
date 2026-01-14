#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct {
    // acceleration history: a_{k-2}, a_{k-1}
    float ax[2], ay[2], az[2]; 

    // position history: p_{k-2}, p_{k-1}
    float px[2], py[2], pz[2];

    float dt; // sample time [s]
} acc_pos;

void init_acc_pos(acc_pos *gyro);
void step(acc_pos *gyro);
void shift(acc_pos *gyro);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif

