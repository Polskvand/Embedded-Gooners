#include "FUNK.c"
#include <stdio.h>

int main(){
    acc_pos gyro;
    init_acc_pos(&gyro);
    printf("%4f, %4f\n", gyro.ax[0], gyro.ax[1]);

    for(int i = 0; i <= 10; i++){
        printf("%d\n", i);
        shift(&gyro.ax[0], i);
        step(&gyro.ax[0], &gyro.px, &gyro.vx, &gyro.dt);
        printf("Acc: (%4f, %4f) \t, vx: %4f\t px: %4f\n", gyro.ax[0], gyro.ax[1], gyro.vx, gyro.px);

    }
    return 0;
}