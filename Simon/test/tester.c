#include "FUNK.h"
#include <stdio.h>

int main(){

    acc_pos gyro;
    init_acc_pos(&gyro);

    printf("%f", gyro.ax);


    float t = 0.0f;
    float dt = 0.01f;

    float screenX = 0;
    float screenY = 0;

    printf("Simulerings data\n");
    for (int i = 0; i < 100; i++) {
        printf("\n%d\n", i);
        acc_sim(t, &gyro.ax, &gyro.ay, &gyro.az);

        step(&gyro.px, &gyro.ax, &gyro.vx, dt);
        step(&gyro.py, &gyro.ay, &gyro.vy, dt);

        mapToScreen(&screenX, &screenY, &gyro.px, &gyro.py, 0.278, 0.059);

        printf("ax : %f\n", gyro.ax);
        printf("vx : %f\n", gyro.vx);
        printf("px : %f\n", gyro.px);
        printf("spx: %f\n", screenX);

        printf("\n");
        
        printf("ay : %f\n", gyro.ay);
        printf("vy : %f\n", gyro.vy);
        printf("py : %f\n", gyro.py);
        printf("spy: %f\n", screenY);

        t += dt;
    }



    

    return 0;
}