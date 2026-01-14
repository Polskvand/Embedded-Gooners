#include "FUNKTIONER.h"
#include <stdio.h>

int main(){
    acc_pos gyro;
    init_acc_pos(&gyro);
    for(int i = 0; i <= 1; i++){
        printf("a[%d](x,y,z): (%.3f, %.3f, %.3f)\n",i-2, gyro.ax[i], gyro.ay[i], gyro.az[i]);
    }
    printf("\n");
    return 0;
}