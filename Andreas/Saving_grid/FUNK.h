#ifndef FUNK
#define FUNK

#include <stdint.h>
#include <stdbool.h>

// uint8_t oled_buffer[128 * (64 / 8)] = {0};


typedef struct {float pos, vel, acc;} Axis_state;
typedef struct {int x, y;} Point;
typedef struct {
    Axis_state x, y, z;
    Point last_cell;
    uint8_t oled_buffer[128 * (64 / 8)];
} Gyro_state;


void init_Axis(Axis_state *axis);
void init_Gyro(Gyro_state *gs);
void step_Axis(Axis_state *axis, float acc_new);
void setPixel(Gyro_state *gs, bool on);
int select_G(int current);

#endif