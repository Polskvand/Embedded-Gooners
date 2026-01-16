#include "FUNKTIONER.h"
#include "PRE_DEFINE.h"

#include <stdio.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_reg.h"

void init_gyro(gyro_state *gyro, float ax0, float ay0, float az0){
    *gyro = (gyro_state){0};
    gyro->x.a = ax0;
    gyro->y.a = ay0;
    gyro->z.a = az0;
}

void step(Axis_state *axis, float acc_new){
    float dt = 0.01;
    float vel_old = axis->v;
    axis->v = vel_old + 0.5f * dt * (acc_new + axis->a);
    axis->p = axis->p + dt * vel_old + 0.25f*dt*dt*(axis->a + acc_new);
    axis->a = acc_new;
}

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

void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
    i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

void mpu_write_reg(uint8_t reg, uint8_t val) {
     i2c_master_write_to_device(I2C_PORT, MPU_ADDR, (uint8_t[]){reg, val}, 2, pdMS_TO_TICKS(100));
}


int16_t be16(const uint8_t *p) {
    return (int16_t)((p[0] << 8) | p[1]);
}