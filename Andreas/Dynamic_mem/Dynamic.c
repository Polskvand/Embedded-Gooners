#include "FUNKTIONER.h"

#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <unistd.h>


int main(){
    gyro_state g;
    init_gyro(&g);

    float ax = 0.0f, ay = 0.0f;
    const float ACC = 0.9f;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    while (1) {
        int ch = getch();

        ax = 0.0f;
        ay = 0.0f;

        switch (ch) {
            case KEY_UP:    ay += ACC; break;
            case KEY_DOWN:  ay -= ACC; break;
            case KEY_RIGHT: ax += ACC; break;
            case KEY_LEFT:  ax -= ACC; break;
            case 'q':       goto exit;
        }

        step_axis(&g.x, ax);
        step_axis(&g.y, ay);

        log_point(&g);

        clear();
        mvprintw(0, 0, "Use arrow keys to move, q to quit");
        mvprintw(2, 0, "Position:");
        mvprintw(3, 0, "x = %.2f (int %d)", g.x.p, (int)g.x.p);
        mvprintw(4, 0, "y = %.2f (int %d)", g.y.p, (int)g.y.p);

        mvprintw(6, 0, "Last cell: (%d, %d)", g.last_cell.x, g.last_cell.y);
        mvprintw(7, 0, "Path len: %d", g.path_len);

        refresh();
        usleep(100000);
    }

exit:
    clear();
    mvprintw(0, 0, "2D Path history (press any key to exit)");
    mvprintw(2, 0, "len=%d:", g.path_len);

    int row = 3;
    for (int i = 0; i < g.path_len; i++) {
        if (row >= LINES - 1) break;
        mvprintw(row++, 2, "(%d, %d)", g.path[i].x, g.path[i].y);
    }

    refresh();
    nodelay(stdscr, FALSE);
    getch();

    free(g.path);
    endwin();
    return 0;
}

