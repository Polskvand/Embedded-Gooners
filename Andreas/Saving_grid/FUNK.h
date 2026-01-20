#ifndef FUNK
#define FUNK

#include <stdint.h>

typedef struct {float pos, vel, acc;} Axis_state;
typedef struct {int x, y;} Point;
typedef struct {
    Axis_state x, y, z;
    Point last_cell;
    uint8_t visited[128 * 64];
} Gyro_state;

void init_Axis(Axis_state *axis);
void init_Gyro(Gyro_state *gs);
void step_Axis(Axis_state *axis, float acc_new);
int get_Idx(int x, int y);
void log_Point(Gyro_state *gs);
int select_G(int current);


#endif