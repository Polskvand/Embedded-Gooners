#include "FUNK.h"
#include <stdio.h>

int main(){

    acc_pos gyro;
    init_acc_pos(&gyro);

    printf("%f", gyro.ax);


    float t = 0.0f;
    float dt = 0.01f;

    printf("Simulerings data\n");
    for (int i = 0; i < 100; i++) {
        printf("\n%d\n", i);
        acc_sim(t, &gyro.ax, &gyro.ay, &gyro.az);


        step(&gyro.px, &gyro.ax, &gyro.vx, dt);
        printf("ax : %f\n", gyro.ax);
        printf("vx : %f\n", gyro.vx);
        printf("px : %f\n", gyro.px);

        t += dt;
    }




    

    return 0;
}