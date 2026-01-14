#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct{
    // Previus acceleration measurement for each axis: a_[-2], a_[-1]
    float ax[2], ay[2], az[2];

    // Previus position measurement for each axis: p_[-2], p_[-1], p_[0], p_[1]
    // Position p[1] bliver brugt som midlertidlig
    float px[4], py[4], pz[4];
} acc_pos;

void init_acc_pos(acc_pos *gyro);
void update_step(acc_pos *gyro, float ax_new, float ay_new, float az_new);

void acc_sim(float t, float *ax, float *ay, float *az);

#endif