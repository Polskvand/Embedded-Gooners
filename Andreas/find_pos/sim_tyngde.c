#include "FUNKTIONER.h"
#include <stdio.h>

int main(){
    acc_pos gyro;
    init_acc_pos(&gyro);

    float ax, ay, az;
    float t = 0.0f;
    float dt = 0.01f;

    printf("Simulerings data\n");
    for (int i = 0; i < 100; i++) {
        acc_sim(t, &ax, &ay, &az);
        printf("%4f, %4f, %4f\n", ax, ay, az);
        t += dt;
    }


    printf("\n");
    return 0;
}
