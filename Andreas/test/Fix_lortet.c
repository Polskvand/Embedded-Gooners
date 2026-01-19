#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>


int mapping(float x, float in_min, float in_max, float out_min, float out_max){
    return  (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float clamp(float x, float min, float max){
    if (x < min) return min;
    if (x > max) return max;
    return x;
}


int main(){
    
    for (float x = -10.0; x <= 10.f; x += 0.1f) {
        float ay_g_int = mapping(clamp(x, -1, 1), -1, 1, 0.0, 511);
        printf("%f\n", ay_g_int);
    }

    return 0;
}
//     bool rotation_acceleration_swtich = false;
//     float ax_g, ay_g, az_g; //Range [-2, 2]
//     float gy_dps, gx_dps; //Range [-250, 250]



// // TODO: Turn into a function or switch thingy 
//         if (rotation_acceleration_swtich) {
//             // In acceleration mode
//             // FORWARD_BACKWARDS
//             if (ay_g < 0) { // Alt mindre end 2 skal også forstås som max
//                 int ay_g_int = mapping(ay_g, -1.0, 0.0, 0.0, 511.0);
//                 // set_LED(BACKWARD_CHANNEL, 511 - ay_g_int);
//                 // set_LED(FORWARD_CHANNEL, 1);
//             } else {
//                 int ay_g_int = mapping(ay_g, 0.0, 1.0, 0.0, 511.0);
//                 // set_LED(BACKWARD_CHANNEL, 1);
//                 // set_LED(FORWARD_CHANNEL, ay_g_int);
//             }
            
//             // LEFT-RIGHT
//             if (ax_g < 0) {
//                 int ax_g_int = mapping(ax_g, -1.0, 0.0, 0.0, 511.0);
//                 // set_LED(LEFT_CHANNEL, 511 - ax_g_int);
//                 // set_LED(RIGHT_CHANNEL, 1);
//             } else {
//                 int ax_g_int = mapping(ax_g, 0.0, 1.0, 0.0, 511.0);
//                 // set_LED(LEFT_CHANNEL, 1);
//                 // set_LED(RIGHT_CHANNEL, ax_g_int);
//             }
//         } else {
//             // In rotation mode
//             // FORWARD-BACK
//             if (gy_dps < 0) {
//                 int gy_dps_int = mapping(gy_dps, -250.0, 0.0, 0.0, 511.0);
//                 // set_LED(LEFT_CHANNEL, 511 - gy_dps_int);
//                 // set_LED(RIGHT_CHANNEL, 1);
//             } else {
//                 int gy_dps_int = mapping(gy_dps, 0.0, 250.0, 0.0, 511.0);
//                 // set_LED(LEFT_CHANNEL, 1);
//                 // set_LED(RIGHT_CHANNEL, gy_dps_int);
//             }
            
//             // LEFT-RIGHT
//             if (gx_dps < 0) {
//                 int gx_dps_int = mapping(gx_dps, -250.0, 0.0, 0.0, 511.0);
//                 // set_LED(FORWARD_CHANNEL, 511 - gx_dps_int);
//             } else {
//                 int gx_dps_int = mapping(gx_dps, 0.0, 250.0, 0.0, 511.0);
//                 // set_LED(FORWARD_CHANNEL, 1);
//                 // set_LED(BACKWARD_CHANNEL, gx_dps_int);
//             }
//         }
// }
