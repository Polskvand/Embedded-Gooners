#include "FUNK.h"
#include <stdio.h>

int main(){

    acc_pos gyro;
    init_acc_pos(&gyro);

    printf("%f", gyro.ax[0]);


    float ax, ay, az;
    float t = 0.0f;
    float dt = 0.01f;

    printf("Simulerings data\n");
    for (int i = 0; i < 100; i++) {
        printf("%d\n", i);
        acc_sim(t, &ax, &ay, &az);

        shift(&gyro.ax, ax);
        printf("ax1 %f, ax2 %f\n", gyro.ax[0],gyro.ax[1]);

        gyro.px += (gyro.ax[0]*dt + gyro.ax[1]*dt)*dt;
        printf("pos x : %f\n", gyro.px);


        t += dt;
    }




    

    return 0;
}