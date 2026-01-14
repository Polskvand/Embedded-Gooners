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
        printf("%d\n", i);
        acc_sim(t, &ax, &ay, &az);

        // printf("t=%.2f  ax=%.2f  ay=%.2f  az=%.2f\n", t, ax, ay, az);
        update_step(&gyro, ax, ay, az);
        printf("a[-2]: (%.3f,%.3f,%.3f)\ta[-1]: (%.3f,%.3f,%.3f)\n", gyro.ax[0], gyro.ay[0], gyro.az[0], gyro.ax[1], gyro.ay[1], gyro.az[1]);
        for(int j = 0; j<=3; j++){
            printf("p[%d]: (%.3f,%.3f,%.3f)\t", j-2, gyro.px[j], gyro.py[j], gyro.pz[j]);
        }
        printf("\n\n");
        t += dt;
    }


    printf("\n");
    return 0;
}
