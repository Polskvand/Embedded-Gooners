#include "FUNK.h"
#include <stdio.h>

int main(){
    acc_pos gyro;
    init_acc_pos(&gyro);
    step(&gyro);
    printf("\n");
    shift(&gyro);
    return 0;
}