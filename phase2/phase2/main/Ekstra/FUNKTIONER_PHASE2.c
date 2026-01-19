#include "FUNKTIONER_PHASE2.h"

void acc_moving_avg_update(acc_pos *f, float ax, float ay, float az, float *ax_out, float *ay_out, float *az_out){
    // Remove oldest sample
    f->ax_sum -= f->ax_buf[f->index];
    f->ay_sum -= f->ay_buf[f->index];
    f->az_sum -= f->az_buf[f->index];

    // Insert new sample
    f->ax_buf[f->index] = ax;
    f->ay_buf[f->index] = ay;
    f->az_buf[f->index] = az;

    f->ax_sum += ax;
    f->ay_sum += ay;
    f->az_sum += az;

    f->index++;
    if (f->index >= AVG_WINDOW) {
        f->index = 0;
        f->filled = true;
    }

    int divisor = f->filled ? AVG_WINDOW : f->index;

    *ax_out = f->ax_sum / divisor;
    *ay_out = f->ay_sum / divisor;
    *az_out = f->az_sum / divisor;
}

void mpu_write_reg(uint8_t reg, uint8_t val) {
  i2c_master_write_to_device(I2C_PORT, MPU_ADDR, (uint8_t[]){reg, val}, 2, pdMS_TO_TICKS(100));
}

void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

int16_t be16(const uint8_t *p) {
    return (int16_t)((p[0] << 8) | p[1]);
}

int mapping_RGB(float x, float in_min, float in_max, float out_min, float out_max){
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void startup() {
    // turnOff();
    printf("-- STARTUP SEQUENCE --\n");

    vTaskDelay(pdMS_TO_TICKS(2000));

    // FORWARD
    set_LED(FORWARD_CHANNEL, 511);
    printf("FORWARD\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_LED(FORWARD_CHANNEL, 0);

    // BACK
    set_LED(BACKWARD_CHANNEL, 511);
    printf("BACKWARD\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_LED(BACKWARD_CHANNEL, 0);

    // LEFT
    set_LED(LEFT_CHANNEL, 511);
    printf("LEFT\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_LED(LEFT_CHANNEL, 0);

    // RIGHT
    set_LED(RIGHT_CHANNEL, 511);
    printf("RIGHT\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
    set_LED(RIGHT_CHANNEL, 0);

    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("-- DONE --\n");
    vTaskDelay(pdMS_TO_TICKS(1000));
}

void calibration(float *ax_ptr, float *ay_ptr, float *az_ptr, float *temp_ptr, float *gx_ptr, float *gy_ptr, float *gz_ptr){
    float ax = 0.0, ay = 0.0, az = 0.0, temp = 0.0, gx = 0.0, gy = 0.0, gz = 0.0;

    for (int i = 0; i < 10000; i++) {
        uint8_t buf[14];
        mpu_read_reg(REG_ACCEL_XOUT, buf, sizeof(buf));

        ax +=   be16(&buf[0]) / 16384.0f;
        ay +=   be16(&buf[2]) / 16384.0f;
        az +=   be16(&buf[4]) / 16384.0f;
        temp += be16(&buf[6]) / 340.0f + 36.53f;
        gx +=   be16(&buf[8]) / 131.0f;
        gy +=   be16(&buf[10]) / 131.0f;
        gz +=   be16(&buf[12]) / 131.0f;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *ax_ptr = ax / 10000.0f;
    *ay_ptr = ay / 10000.0f;
    *az_ptr = az / 10000.0f;
    *temp_ptr = temp / 10000.0f;
    *gx_ptr = gx / 10000.0f;
    *gy_ptr = gy / 10000.0f;
    *gz_ptr = gz / 10000.0f;
}

void interrupt_handler(){
    uint64_t new_count;
    gptimer_get_raw_count(gptimer, &new_count);
    if ((new_count - count) > 500000) {
        rot_led_level = !rot_led_level;
        rotation_acceleration_swtich = !rotation_acceleration_swtich;
        count = new_count;
    }    
}

void set_LED(int CHANNEL, int value) {
    ledc_set_duty(LEDC_MODE, CHANNEL, value);
    ledc_update_duty(LEDC_MODE, CHANNEL);
}

