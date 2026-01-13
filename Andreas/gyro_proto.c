#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     4000

#define MPU_ADDR        0x68

// MPU6050 registre
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT  0x3B  // start for accel/gyro burst read

static const char *TAG = "MPU6050";

void i2c_master_init() {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_PIN,
      .scl_io_num = I2C_SCL_PIN,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master.clk_speed = I2C_FREQ_HZ,
  };
  i2c_param_config(I2C_PORT, &conf);
  i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

void mpu_write_reg(uint8_t reg, uint8_t val) {
  i2c_master_write_to_device(I2C_PORT, MPU_ADDR, (uint8_t[]){reg, val}, 2, pdMS_TO_TICKS(100));
}

void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

int16_t be16(const uint8_t *p) {  // big-endian to int16
    return (int16_t)((p[0] << 8) | p[1]);
}

void app_main(void) {
    i2c_master_init();

    uint8_t who = 0;
    mpu_read_reg(REG_WHO_AM_I, &who, 1);

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

        printf("A[g]  x=%.3f y=%.3f z=%.3f | G[dps] x=%.2f y=%.2f z=%.2f | T=%.2fC\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
