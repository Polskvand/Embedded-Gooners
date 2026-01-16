#ifndef FUNKTIONER_PHASE2
#define FUNKTIONER_PHASE2

#include "LIBRARIES_PHASE2.h"

void acc_moving_avg_update(acc_pos *f, float ax, float ay, float az, float *ax_out, float *ay_out, float *az_out);

void mpu_write_reg(uint8_t reg, uint8_t val);

void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len);

int16_t be16(const uint8_t *p);

int mapping_RGB(float x, float in_min, float in_max, float out_min, float out_max);

void startup();

void calibration(float *ax_ptr, float *ay_ptr, float *az_ptr, float *temp_ptr, float *gx_ptr, float *gy_ptr, float *gz_ptr);

void interrupt_handler();

void set_LED(int CHANNEL, int value);

#endif