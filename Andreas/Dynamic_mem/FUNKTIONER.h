#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct {
    float p, v, a;
} Axis_state;

typedef struct {
    int x, y;
} Point;

typedef struct {
    Axis_state x, y, z;

    Point *path;
    int path_len, path_cap;
    Point last_cell;
} gyro_state;

void init_axis(Axis_state *axis);
void init_path(gyro_state *g);
void init_gyro(gyro_state *g);
void step_axis(Axis_state *axis, float acc_new);
void log_point(gyro_state *g);


#endif