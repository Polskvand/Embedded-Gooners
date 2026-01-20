#include "FUNKTIONER.h"
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <unistd.h>

void init_axis(Axis_state *axis){
    axis->p = axis->v = axis->a = 0.0f;
}

void init_path(gyro_state *g){
    g->path_len = 0;
    g->path_cap = 16;
    g->path = malloc(g->path_cap * sizeof(Point));
    g->last_cell = (Point){ (int)g->x.p, (int)g->y.p };
    g->path[g->path_len++] = g->last_cell;
}

void init_gyro(gyro_state *g){
    init_axis(&g->x);
    init_axis(&g->y);
    init_axis(&g->z);
    init_path(g);
}

void step_axis(Axis_state *axis, float acc_new){
    float dt = 1.0f;
    float vel_old = axis->v * 0.95f;

    axis->v = vel_old + 0.5f * dt * (acc_new + axis->a);
    axis->p = axis->p + dt * vel_old + 0.25f * dt * dt * (axis->a + acc_new);
    axis->a = acc_new;
}

void log_point(gyro_state *g){
    Point cell = { (int)g->x.p, (int)g->y.p };

    if (cell.x == g->last_cell.x && cell.y == g->last_cell.y) return;

    if (g->path_len >= g->path_cap) {
        g->path_cap *= 2;
        g->path = realloc(g->path, g->path_cap * sizeof(Point));
    }

    g->path[g->path_len++] = cell;
    g->last_cell = cell;
}