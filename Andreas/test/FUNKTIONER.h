#ifndef FUNKTIONER
#define FUNKTIONER

typedef struct { float p, v, a; } Axis_state;
typedef struct { int x, y; } Point;

typedef struct {
    Axis_state x, y, z;
    Point *path, last_cell; 
    int path_len, path_cap, path_head;
} gyro_state;

void axis_zero(Axis_state *s);
void init_path(gyro_state *g);
void path_free(gyro_state *g);
void path_push(gyro_state *g, Point p);
Point path_get(const gyro_state *g, int i);
void init_gyro(gyro_state *g);
void step_axis(Axis_state *axis, float acc_new);
void log_point(gyro_state *g);


#endif