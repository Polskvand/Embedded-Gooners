#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct{
    // Previus acceleration measurement for each axis: a_[-2], a_[-1]
    float ax[2], ay[2], az[2];

    // Previus position measurement for each axis: p_[-2], p_[-1]
    float px[2], py[2], pz[2];
} acc_pos;

void init_acc_pos(acc_pos *gyro);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif