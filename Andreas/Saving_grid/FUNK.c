#include "FUNK.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ncurses.h>
#include <unistd.h>
#include <stdint.h>


void init_Axis(Axis_state *axis) {axis->pos = axis->vel = axis->acc = .0f;}

void init_Gyro(Gyro_state *gs){
    init_Axis(&gs->x); init_Axis(&gs->y); init_Axis(&gs->z);
    memset(gs->visited, 0, sizeof(gs->visited));
    gs->last_cell = (Point){(int)gs->x.pos, (int)gs->y.pos};
}

void step_Axis(Axis_state *axis, float acc_new){
    float dt = 1.0f;
    float vel_old = axis->vel * 0.95f;
    axis->vel = vel_old + 0.5f * dt * (acc_new + axis->acc);
    axis->pos = axis->pos + dt * vel_old + 0.25f * dt * dt * (axis->acc + acc_new);
    axis->acc = acc_new;
}

int get_Idx(int x, int y){
    return y * 128 + x;
}

void log_Point(Gyro_state *gs){
    if (gs->x.pos == gs->last_cell.x && gs->y.pos == gs->last_cell.y) return;
    int idx = get_Idx(gs->x.pos, gs->y.pos);
    if(gs->visited[idx]) gs->visited[idx] = 1;
}

int select_G(int current){
    int sel = current;

    while (1){
        clear();
        mvprintw(0, 0, "Select player (LEFT/RIGHT or UP/DOWN). ENTER = choose, ESC = cancel");
        mvprintw(2, 0, "Current selection: Player %d", sel + 1);
        refresh();

        int ch = getch();
        switch (ch){
            case KEY_LEFT:
            case KEY_UP:
                sel = (sel + 3 - 1) % 3;
                break;

            case KEY_RIGHT:
            case KEY_DOWN:
                sel = (sel + 1) % 3;
                break;

            case 10:
            case KEY_ENTER:
                return sel;

            case 27:
                return current;
        }
        usleep(20000);
    }
}

