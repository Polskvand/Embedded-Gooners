#include <stdio.h>
#include <stdint.h>

#include "PRE_DEFINE.h"
#include "FUNKTIONER.h"

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"


void app_main(void) {
    i2c_master_init();
    gyro_state gyro;
    init_gyro(&gyro, 0, 0, 1);


    // Væk MPU'en (clear sleep bit)
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    while (1) {
        uint8_t buf[14];
        mpu_read_reg(REG_ACCEL_XOUT, buf, sizeof(buf));


        int16_t ax = be16(&buf[0]);
        int16_t ay = be16(&buf[2]);
        int16_t az = be16(&buf[4]);
        int16_t temp = be16(&buf[6]);
        int16_t gx = be16(&buf[8]);
        int16_t gy = be16(&buf[10]);
        int16_t gz = be16(&buf[12]);

        // Standard sensitivitet når den står i default (±2g og ±250°/s):
        // Accel: 16384 LSB/g
        // Gyro : 131 LSB/(°/s)
        float ax_g = ax / 16384.0f;
        float ay_g = ay / 16384.0f;
        float az_g = az / 16384.0f;

        float gx_dps = gx / 131.0f;
        float gy_dps = gy / 131.0f;
        float gz_dps = gz / 131.0f;

        // Temperatur: (temp/340) + 36.53 (ifølge datasheet)
        float temp_c = (temp / 340.0f) + 36.53f;

        (void)gx_dps;
        (void)gy_dps;
        (void)gz_dps;
        (void)temp_c;
        
        step(&gyro.x, ax_g);
        step(&gyro.y, ay_g);
        step(&gyro.z, az_g);

        printf("a:(%3f %3f %3f)\t\tv:(%3f %3f %3f)\t\tp:(%3f %3f %3f)\t\n", gyro.x.a, gyro.y.a, gyro.z.a,
                                                                            gyro.x.v, gyro.y.v, gyro.z.v,
                                                                            gyro.x.p, gyro.y.p, gyro.z.p);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
