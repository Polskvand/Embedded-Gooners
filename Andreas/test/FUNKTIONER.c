#include "FUNKTIONER.h"
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <unistd.h>

void axis_zero(Axis_state *s){ s->p = s->v = s->a = 0.0f; }

void init_path(gyro_state *g){
    g->path_cap = 500;
    g->path = malloc((size_t)g->path_cap * sizeof(*g->path));
    g->path_len = g->path_head = 0;
}

void init_gyro(gyro_state *g){
    axis_zero(&g->x); axis_zero(&g->y); axis_zero(&g->z);
    g->path = NULL; g->path_len = g->path_cap = g->path_head = 0;
    g->last_cell = (Point){(int)g->x.p, (int)g->y.p};
    path_push(g, g->last_cell);
}

void path_free(gyro_state *g){
    free(g->path);
    *g = (gyro_state){0};
}

void path_push(gyro_state *g, Point p){
    if (!g->path) init_path(g);
    int idx = (g->path_head + g->path_len) % g->path_cap;
    g->path[idx] = p;
    if (g->path_len < g->path_cap) g->path_len++;
    else g->path_head = (g->path_head + 1) % g->path_cap;
}

Point path_get(const gyro_state *g, int i){
    return g->path[(g->path_head + i) % g->path_cap];
}


void step_axis(Axis_state *s, float a){
    const float dt = 1.0f, v = s->v * 0.95f;
    s->v = v + 0.5f * dt * (a + s->a);
    s->p += dt * v + 0.25f * dt * dt * (s->a + a);
    s->a = a;
}

void log_point(gyro_state *g){
    Point c = {
        ((int)g->x.p) & 127,
        ((int)g->y.p) & 63
    };

    if (c.x != g->last_cell.x || c.y != g->last_cell.y)
        path_push(g, g->last_cell = c);
}
