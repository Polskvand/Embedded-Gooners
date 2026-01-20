#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ncurses.h>
#include <unistd.h>
#include <stdint.h>


typedef struct {float pos, vel, acc;} Axis_state;
typedef struct {int x, y;} Point;
typedef struct {
    Axis_state x, y, z;
    Point last_cell;
    uint8_t visited[128 * 64];
} Gyro_state;

inline void init_Axis(Axis_state *axis) {axis->pos = axis->vel = axis->acc = .0f;}

inline void init_Gyro(Gyro_state *gs){
    init_Axis(&gs->x); init_Axis(&gs->y); init_Axis(&gs->z);
    memset(gs->visited, 0, sizeof(gs->visited));
    gs->last_cell = (Point){(int)gs->x.pos, (int)gs->y.pos};
}

void step_axis(Axis_state *axis, float acc_new){
    float dt = 1.0f;
    float vel_old = axis->vel * 0.95f;
    axis->vel = vel_old + 0.5f * dt * (acc_new + axis->acc);
    axis->pos = axis->pos + dt * vel_old + 0.25f * dt * dt * (axis->acc + acc_new);
    axis->acc = acc_new;
}

inline int get_idx(int x, int y){
    return y * 128 + x;
}

void log_point(Gyro_state *gs){
    if (gs->x.pos == gs->last_cell.x && gs->y.pos == gs->last_cell.y) return;
    int idx = get_idx(gs->x.pos, gs->y.pos);
    if(gs->visited[idx]) gs->visited[idx] = 1;
}

int select_g(int current){
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

int main(void){
    Gyro_state g[3];
    for(int i = 0; i < 3; i++){
        init_Gyro(&g[i]);
    }

    int active = 0;

    float ax = 0.0f, ay = 0.0f;
    const float ACC = 0.9f;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    int running = 1;
    while (running){
        int ch = getch();
        ax = 0.0f; ay = 0.0f;

        switch (ch) {
            case KEY_UP:    ay += ACC; break;
            case KEY_DOWN:  ay -= ACC; break;
            case KEY_RIGHT: ax += ACC; break;
            case KEY_LEFT:  ax -= ACC; break;

            case ' ':
                nodelay(stdscr, FALSE);
                active = select_g(active);
                nodelay(stdscr, TRUE);
                break;

            case 'q':
                running = 0;
                break;
        }

        // opdater kun den aktive spiller
        step_axis(&g[active].x, ax);
        step_axis(&g[active].y, ay);
        log_point(&g[active]);

        clear();
        mvprintw(0, 0, "Arrows move | SPACE select player | q quit");
        mvprintw(1, 0, "Active: Player %d", active + 1);

        mvprintw(3, 0, "Position:");
        mvprintw(4, 0, "x = %.2f (int %d)", g[active].x.pos, (int)g[active].x.pos);
        mvprintw(5, 0, "y = %.2f (int %d)", g[active].y.pos, (int)g[active].y.pos);


        refresh();
        usleep(100000);
    }

    endwin();
    return 0;
}
