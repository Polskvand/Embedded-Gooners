#include "FUNK.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <ncurses.h>
#include <unistd.h>
#include <stdint.h>

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
                active = select_G(active);
                nodelay(stdscr, TRUE);
                break;

            case 'q':
                running = 0;
                break;
        }

        step_Axis(&g[active].x, ax);
        step_Axis(&g[active].y, ay);

        setPixel(&g[active], 1);


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
