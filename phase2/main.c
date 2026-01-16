#include "INITIALISER_PHASE2.h"
#include "FUNKTIONER_PHASE2.h"


void app_main(void) {
    gpio_configure();

    int acc_range = 2;

    float acc_scale;
    switch (acc_range) {
        case 4:
            mpu_write_reg(REG_ACCEL_CONFIG, 0x08);
            acc_scale = 8192.0f;
            break;
        case 8:
            mpu_write_reg(REG_ACCEL_CONFIG, 0x10);
            acc_scale = 4096.0f;
            break;
        case 16:
            mpu_write_reg(REG_ACCEL_CONFIG, 0x18);
            acc_scale = 2048.0f;
            break;
        default:
            mpu_write_reg(REG_ACCEL_CONFIG, 0x00);
            acc_scale = 16384.0f;
            break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));


    // Startup sequence
    startup();

    // Calibration
    printf("-- Beginning calibration --\n");
    float ax_cal = 0.0, ay_cal = 0.0, az_cal = 0.0, temp_cal = 0.0, gx_cal = 0.0, gy_cal = 0.0, gz_cal = 0.0;
    calibration(&ax_cal, 
                &ay_cal, 
                &az_cal, 
                &temp_cal, 
                &gx_cal, 
                &gy_cal, 
                &gz_cal);

    printf("-- Calibration complete-- \nAverage values:\n");
    printf("ax_cal = %.2f, ay_cal = %.2f, az_cal = %.2f, temp_cal = %.2f, gx_cal = %.2f, gy_cal = %.2f, gz_cal = %.2f\n\n", 
        ax_cal, ay_cal, az_cal, temp_cal, gx_cal, gy_cal, gz_cal);
    
    vTaskDelay(pdMS_TO_TICKS(2000));

    acc_pos acc_filter;
    init_acc_pos(&acc_filter);


    while (1) {
        gpio_set_level(MODE_PIN, rot_led_level);

        uint8_t buf[14];
        mpu_read_reg(REG_ACCEL_XOUT, buf, sizeof(buf));

        int16_t ax =   be16(&buf[0]);
        int16_t ay =   be16(&buf[2]);
        int16_t az =   be16(&buf[4]);
        int16_t temp = be16(&buf[6]);
        int16_t gx =   be16(&buf[8]);
        int16_t gy =   be16(&buf[10]);
        int16_t gz =   be16(&buf[12]);

        // Standard sensitivitet når den står i default (±2g og ±250°/s):
        // Accel: 16384 LSB/g
        // Gyro : 131 LSB/(°/s)

        float ax_raw = -az / acc_scale; // Changing x and z to align board with component orientation
        float ay_raw =  ay / acc_scale;
        float az_raw =  ax / acc_scale;

        float ax_g, ay_g, az_g;

        acc_moving_avg_update(&acc_filter,
                            ax_raw - (-az_cal), ay_raw - ay_cal, az_raw - ax_cal + 1, // az_raw - az_cal + 1 to orient z axis up
                            &ax_g, &ay_g, &az_g);

        float gx_dps = (-gz / 131.0f) - (-gz_cal);
        float gy_dps =  (gy / 131.0f) - gy_cal;
        float gz_dps =  (gx / 131.0f) - gx_cal;

        // Temperatur: (temp/340) + 36.53 (ifølge datasheet)
        float temp_c = (temp / 340.0f) + 36.53f; // set - temp_cal for current temperature deviation instead of reading

        if (rotation_acceleration_swtich) {
            // In acceleration mode
            // FORWARD_BACKWARDS
            if (ay_g < 0) {
                int ay_g_int = mapping_RGB(ay_g, -1.0, 0.0, 0.0, 511.0);
                set_LED(BACKWARD_CHANNEL, 511 - ay_g_int);
                set_LED(FORWARD_CHANNEL, 1);
            } else {
                int ay_g_int = mapping_RGB(ay_g, 0.0, 1.0, 0.0, 511.0);
                set_LED(BACKWARD_CHANNEL, 1);
                set_LED(FORWARD_CHANNEL, ay_g_int);
            }
            
            // LEFT-RIGHT
            if (ax_g < 0) {
                int ax_g_int = mapping_RGB(ax_g, -1.0, 0.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 511 - ax_g_int);
                set_LED(RIGHT_CHANNEL, 1);
            } else {
                int ax_g_int = mapping_RGB(ax_g, 0.0, 1.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 1);
                set_LED(RIGHT_CHANNEL, ax_g_int);
            }
        } else {
            // In rotation mode
            // FORWARD-BACK
            if (gy_dps < 0) {
                int gy_dps_int = mapping_RGB(gy_dps, -250.0, 0.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 511 - gy_dps_int);
                set_LED(RIGHT_CHANNEL, 1);
            } else {
                int gy_dps_int = mapping_RGB(gy_dps, 0.0, 250.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 1);
                set_LED(RIGHT_CHANNEL, gy_dps_int);
            }
            
            // LEFT-RIGHT
            if (gx_dps < 0) {
                int gx_dps_int = mapping_RGB(gx_dps, -250.0, 0.0, 0.0, 511.0);
                set_LED(FORWARD_CHANNEL, 511 - gx_dps_int);
            } else {
                int gx_dps_int = mapping_RGB(gx_dps, 0.0, 250.0, 0.0, 511.0);
                set_LED(FORWARD_CHANNEL, 1);
                set_LED(BACKWARD_CHANNEL, gx_dps_int);
            }
        }

        printf("A[g] (x,y,z) = (%5.2f, %5.2f, %5.2f)\t G[dps] (x,y,z) = (%7.2f, %7.2f, %7.2f) \t | T=%5.2fC\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);
    }
}

