#ifndef FUNKTIONER
#define FUNKTIONER

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

typedef struct{
    float p, v, a;
} Axis_state;

typedef struct{
    Axis_state x, y, z;
} gyro_state;

void init_gyro(gyro_state *gyro, float ax0, float ay0, float az0);
void step(Axis_state *axis, float acc_new);
void i2c_master_init();
void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len);
void mpu_write_reg(uint8_t reg, uint8_t val);
int16_t be16(const uint8_t *p);

#endif