#include <stdio.h>
#include "FUNKTIONER.h"

int main(){
    gyro_state gyro;
    init_gyro(&gyro, 0, 0, 1);

    for(int i = 0; i <= 10; i++){
        step(&gyro.x, 0);
        step(&gyro.y, 0);
        step(&gyro.z, 1);
        printf("a:(%3f %3f %3f)\t\tv:(%3f %3f %3f)\t\tp:(%3f %3f %3f)\t\t\n", gyro.x.a, gyro.y.a, gyro.z.a,
                                                                            gyro.x.v, gyro.y.v, gyro.z.v,
                                                                            gyro.x.p, gyro.y.p, gyro.z.p);
    }
}