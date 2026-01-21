#define _POSIX_C_SOURCE 200809L

#include "FUNKTIONER.h"
#include <stdio.h>
#include <stdlib.h>
#include <ncurses.h>
#include <time.h>

#define NUM_USERS 3

int main() {
    gyro_state users[NUM_USERS];
    for (int i = 0; i < NUM_USERS; i++) {
        init_gyro(&users[i]);
    }

    int active_user = 0;
    gyro_state *g = &users[active_user];

    float ax = 0.0f, ay = 0.0f;
    const float ACC = 0.9f;

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);

    struct timespec ts = { .tv_sec = 0, .tv_nsec = 10 * 1000 * 1000 }; // 100ms

    while (1) {
        int ch = getch();

        ax = 0.0f;
        ay = 0.0f;

        switch (ch) {
            case KEY_UP:    ay += ACC; break;
            case KEY_DOWN:  ay -= ACC; break;
            case KEY_RIGHT: ax += ACC; break;
            case KEY_LEFT:  ax -= ACC; break;

            // Skift bruger med 1/2/3
            case '1': active_user = 0; g = &users[active_user]; break;
            case '2': active_user = 1; g = &users[active_user]; break;
            case '3': active_user = 2; g = &users[active_user]; break;

            // Alternativ: TAB til næste bruger
            case '\t':
                active_user = (active_user + 1) % NUM_USERS;
                g = &users[active_user];
                break;

            case 'q': goto exit;
        }

        // Opdater kun den aktive bruger
        step_axis(&g->x, ax);
        step_axis(&g->y, ay);
        log_point(g);

        clear();
        mvprintw(0, 0, "Arrows move | 1/2/3 switch user | TAB next | q quit");
        mvprintw(1, 0, "Active user: %d", active_user + 1);

        mvprintw(3, 0, "Position:");
        mvprintw(4, 0, "x = %.2f (int %d)", g->x.p, (int)g->x.p);
        mvprintw(5, 0, "y = %.2f (int %d)", g->y.p, (int)g->y.p);

        mvprintw(7, 0, "Last cell: (%d, %d)", g->last_cell.x, g->last_cell.y);
        mvprintw(8, 0, "Path len: %d", g->path_len);

        // (valgfrit) vis path-længde for alle brugere
        mvprintw(10, 0, "All users path lens: U1=%d U2=%d U3=%d",
                 users[0].path_len, users[1].path_len, users[2].path_len);

        nanosleep(&ts, NULL);
        refresh();
    }

exit:
    endwin();
    int len0 = users[0].path_len;
    int len1 = users[1].path_len;
    int len2 = users[2].path_len;

    int max_len = len0;
    if (len1 > max_len) max_len = len1;
    if (len2 > max_len) max_len = len2;

    printf("\n=== Paths ===\n");
    printf("U1 len=%d | U2 len=%d | U3 len=%d\n", len0, len1, len2);

    for (int i = 0; i < max_len; i++) {
        // User 1
        if (i < len0) {
            Point p = path_get(&users[0], i);
            printf("User 1:(%d, %d)\t", p.x, p.y);
        } else {
            printf("User 1:( , )\t");
        }

        // User 2
        if (i < len1) {
            Point p = path_get(&users[1], i);
            printf("User 2:(%d, %d)\t", p.x, p.y);
        } else {
            printf("User 2:( , )\t");
        }

        // User 3
        if (i < len2) {
            Point p = path_get(&users[2], i);
            printf("User 3:(%d, %d)", p.x, p.y);
        } else {
            printf("User 3:( , )");
        }

        printf("\n");
    }
    return 0;
}
