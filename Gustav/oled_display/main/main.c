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

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_5
#define I2C_SCL_PIN     GPIO_NUM_6
#define I2C_FREQ_HZ     400000

#define MPU_ADDR        0x68

#define OLED_ADDR       0x3C

// MPU6050 registre
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT     0x3B
#define REG_ACCEL_CONFIG   0x1C

#define LED_FORWARD     GPIO_NUM_1
#define LED_BACKWARD    GPIO_NUM_9
#define LED_LEFT        GPIO_NUM_7
#define LED_RIGHT       GPIO_NUM_0

#define BUTTON_PIN      GPIO_NUM_4
#define MODE_PIN        GPIO_NUM_19

#define LEDC_MODE       LEDC_LOW_SPEED_MODE

#define FORWARD_CHANNEL     0
#define BACKWARD_CHANNEL    1
#define LEFT_CHANNEL        2
#define RIGHT_CHANNEL       3

#define RGB_TIMER       0
#define RGB_DUTY_RES    9       // 9-bit resolution
#define RGB_FREQUENCY   5000    // 5 kHz PWM
#define AVG_WINDOW      32


#define OLED_WIDTH 128
#define OLED_HEIGHT 64

typedef struct {
    float ax_buf[AVG_WINDOW];
    float ay_buf[AVG_WINDOW];
    float az_buf[AVG_WINDOW];

    float ax_sum;
    float ay_sum;
    float az_sum;

    uint8_t index;
    bool filled;
} acc_pos;

bool rot_led_level = false;
bool rotation_acceleration_swtich = true;
uint64_t count;
gptimer_handle_t gptimer = NULL;
uint8_t oled_buffer[OLED_WIDTH * (OLED_HEIGHT / 8)] = {0};


// Prototype functions
void config_button();
void config_i2c_master();
void config_timer();
void config_mode_led();
void config_direction_channels();
void config_led_timer();
void config_ssd1306();

void init_acc_pos(acc_pos *f);

void acc_moving_avg_update(acc_pos *f, float ax, float ay, float az, float *ax_out, float *ay_out, float *az_out);

void mpu_write_reg(uint8_t reg, uint8_t val);

void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len);

int16_t be16(const uint8_t *p);

int mapping(float x, float in_min, float in_max, float out_min, float out_max);

void calibration(float *ax_ptr, float *ay_ptr, float *az_ptr, float *temp_ptr, float *gx_ptr, float *gy_ptr, float *gz_ptr);

void interrupt_handler();

void set_LED(int CHANNEL, int value);

void ssd1306_cmd(uint8_t cmd);

void oled_update();

void setPixel(uint8_t x, uint8_t y, bool on);

void turn_on_all();

void turn_off_all();

// TODO: More comments (english) - Remove obvious Chad comments
void app_main(void) {
    i2c_driver_delete(I2C_PORT);
    vTaskDelay(pdMS_TO_TICKS(10));

    config_i2c_master();
    config_mode_led();
    config_timer();
    config_button();
    config_led_timer();
    config_direction_channels();
    config_ssd1306();

    gptimer_get_raw_count(gptimer, &count);

    // Force reset to avoid power issues
    mpu_write_reg(REG_PWR_MGMT_1, 0x80);
    vTaskDelay(pdMS_TO_TICKS(10));
    // Wake MPU (clear sleep bit)
    mpu_write_reg(REG_PWR_MGMT_1, 0x00); 
    
    turn_on_all(); // Startup begins

    // Calibration
    printf("-- Beginning calibration --\n");
    float ax_cal = 0.0, ay_cal = 0.0, az_cal = 0.0, temp_cal = 0.0, gx_cal = 0.0, gy_cal = 0.0, gz_cal = 0.0;
    calibration(&ax_cal, &ay_cal, &az_cal, &temp_cal, &gx_cal, &gy_cal, &gz_cal);

    printf("-- Calibration complete-- \nAverage values:\n");
    printf("ax_cal = %.2f, ay_cal = %.2f, az_cal = %.2f, temp_cal = %.2f, gx_cal = %.2f, gy_cal = %.2f, gz_cal = %.2f\n\n", 
        ax_cal, ay_cal, az_cal, temp_cal, gx_cal, gy_cal, gz_cal);

    turn_off_all(); // Startup ends

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

        float acc_scale = 16384.0f;
        float ax_raw = -az / acc_scale; // Changing x and z to align board with component orientation
        float ay_raw =  ay / acc_scale;
        float az_raw =  ax / acc_scale;

        float ax_g, ay_g, az_g;

        acc_moving_avg_update(&acc_filter,
                            ax_raw - (-az_cal), ay_raw - ay_cal, az_raw - ax_cal + 1, // az_raw - ax_cal + 1 to orient z axis up
                            &ax_g, &ay_g, &az_g);

        float gx_dps = (-gz / 131.0f) - (-gz_cal);
        float gy_dps =  (gy / 131.0f) - gy_cal;
        float gz_dps =  (gx / 131.0f) - gx_cal;

        // Temperatur: (temp/340) + 36.53 (ifølge datasheet)
        float temp_c = (temp / 340.0f) + 36.53f; // set - temp_cal for current temperature deviation instead of reading

        // TODO: Turn into a function or switch thingy 
        if (rotation_acceleration_swtich) {
            // In acceleration mode
            // FORWARD_BACKWARDS
            if (ay_g < 0) {
                int ay_g_int = mapping(ay_g, -1.0, 0.0, 0.0, 511.0);
                set_LED(BACKWARD_CHANNEL, 511 - ay_g_int);
                set_LED(FORWARD_CHANNEL, 1);
            } else {
                int ay_g_int = mapping(ay_g, 0.0, 1.0, 0.0, 511.0);
                set_LED(BACKWARD_CHANNEL, 1);
                set_LED(FORWARD_CHANNEL, ay_g_int);
            }
            
            // LEFT-RIGHT
            if (ax_g < 0) {
                int ax_g_int = mapping(ax_g, -1.0, 0.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 511 - ax_g_int);
                set_LED(RIGHT_CHANNEL, 1);
            } else {
                int ax_g_int = mapping(ax_g, 0.0, 1.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 1);
                set_LED(RIGHT_CHANNEL, ax_g_int);
            }
        } else {
            // In rotation mode
            // FORWARD-BACK
            if (gy_dps < 0) {
                int gy_dps_int = mapping(gy_dps, -250.0, 0.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 511 - gy_dps_int);
                set_LED(RIGHT_CHANNEL, 1);
            } else {
                int gy_dps_int = mapping(gy_dps, 0.0, 250.0, 0.0, 511.0);
                set_LED(LEFT_CHANNEL, 1);
                set_LED(RIGHT_CHANNEL, gy_dps_int);
            }
            
            // LEFT-RIGHT
            if (gx_dps < 0) {
                int gx_dps_int = mapping(gx_dps, -250.0, 0.0, 0.0, 511.0);
                set_LED(FORWARD_CHANNEL, 511 - gx_dps_int);
            } else {
                int gx_dps_int = mapping(gx_dps, 0.0, 250.0, 0.0, 511.0);
                set_LED(FORWARD_CHANNEL, 1);
                set_LED(BACKWARD_CHANNEL, gx_dps_int);
            }
        }



        // OLED pixel
        int ay_pixel = mapping(ay_g, -1.0, 1.0, 0.0, 128.0); // Forward-back
        int ax_pixel = mapping(ax_g, -1.0, 1.0, 0.0, 64.0); // Left-right
        setPixel(ay_pixel, ax_pixel, true);    
        oled_update();


        // TODO: Få flushed
        printf("A[g] (x,y,z) = (%5.2f, %5.2f, %5.2f)\t G[dps] (x,y,z) = (%7.2f, %7.2f, %7.2f) \t | T=%5.2fC\n",
               ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);
        // fflush(stdout);
    }



    // TODO: Implement positions with momentum
}



// Functions

// Calibration of gyroscope. Creates average errors
void calibration(float *ax_ptr, 
                 float *ay_ptr, 
                 float *az_ptr, 
                 float *temp_ptr, 
                 float *gx_ptr, 
                 float *gy_ptr, 
                 float *gz_ptr) {

    int n_steps = 10000;
    float ax = 0.0, ay = 0.0, az = 0.0, temp = 0.0, gx = 0.0, gy = 0.0, gz = 0.0;

    for (int i = 0; i < n_steps; i++) {
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

    *ax_ptr     = ax / (float)n_steps;
    *ay_ptr     = ay / (float)n_steps;
    *az_ptr     = az / (float)n_steps;
    *temp_ptr   = temp / (float)n_steps;
    *gx_ptr     = gx / (float)n_steps;
    *gy_ptr     = gy / (float)n_steps;
    *gz_ptr     = gz / (float)n_steps;
}


void init_acc_pos(acc_pos *f) {
    memset(f, 0, sizeof(acc_pos));
}


void config_button() {
    gpio_config_t io_conf_ins = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf_ins);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, interrupt_handler, NULL);
}

void config_mode_led() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);
}

void config_i2c_master() {
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

void config_timer() {
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    gptimer_new_timer(&timer_config, &gptimer);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);
}

void config_direction_channels() {
    ledc_channel_config_t forward_channel_cfg = {
        .gpio_num       = LED_FORWARD,
        .speed_mode     = LEDC_MODE,
        .channel        = FORWARD_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&forward_channel_cfg);

    ledc_channel_config_t backward_channel_cfg = {
        .gpio_num       = LED_BACKWARD,
        .speed_mode     = LEDC_MODE,
        .channel        = BACKWARD_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&backward_channel_cfg);

    ledc_channel_config_t left_channel_cfg = {
        .gpio_num       = LED_LEFT,
        .speed_mode     = LEDC_MODE,
        .channel        = LEFT_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&left_channel_cfg);

    ledc_channel_config_t right_channel_cfg = {
        .gpio_num       = LED_RIGHT,
        .speed_mode     = LEDC_MODE,
        .channel        = RIGHT_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&right_channel_cfg);
}

void config_led_timer() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = RGB_DUTY_RES,
        .timer_num        = RGB_TIMER,
        .freq_hz          = RGB_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
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

int mapping(float x, float in_min, float in_max, float out_min, float out_max){
    return  fmax(fmin(((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min), out_max), 0);
}

void interrupt_handler()
{
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

void acc_moving_avg_update(acc_pos *f, float ax, float ay, float az,
                           float *ax_out, float *ay_out, float *az_out)
{
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


void ssd1306_cmd(uint8_t cmd)
{
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x00, true); // Control byte: command
    i2c_master_write_byte(h, cmd, true);
    i2c_master_stop(h);
    i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
}

void oled_update()
{
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_cmd(0xB0 + page);  // vælg page
        ssd1306_cmd(0x00);         // column low
        ssd1306_cmd(0x10);         // column high

        i2c_cmd_handle_t h = i2c_cmd_link_create();
        i2c_master_start(h);
        i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(h, 0x40, true); // data mode

        for (uint8_t x = 0; x < OLED_WIDTH; x++) {
            i2c_master_write_byte(
                h,
                oled_buffer[page * OLED_WIDTH + x],
                true
            );
        }

        i2c_master_stop(h);
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100)));
        i2c_cmd_link_delete(h);
    }
}

void setPixel(uint8_t x, uint8_t y, bool on)
{
    if (x >= OLED_WIDTH || y >= OLED_HEIGHT)
        return;

    uint8_t page = y / 8;                     // hvilken page
    uint8_t bit  = y % 8;                     // hvilket bit
    uint16_t index = page * OLED_WIDTH + x;   // byte i buffer

    if (on)
        oled_buffer[index] |=  (1 << bit);    // tænd pixel
    else
        oled_buffer[index] &= ~(1 << bit);    // sluk pixel
}


void config_ssd1306()
{
    ssd1306_cmd(0xAE); // Display OFF

    ssd1306_cmd(0xD5);
    ssd1306_cmd(0x80);

    ssd1306_cmd(0xA8);
    ssd1306_cmd(0x3F);

    ssd1306_cmd(0xD3);
    ssd1306_cmd(0x00);

    ssd1306_cmd(0x40);

    ssd1306_cmd(0x8D);
    ssd1306_cmd(0x14); // Charge pump ON

    ssd1306_cmd(0xA1);
    ssd1306_cmd(0xC8);

    ssd1306_cmd(0xDA);
    ssd1306_cmd(0x12);

    ssd1306_cmd(0x81);
    ssd1306_cmd(0x7F);

    ssd1306_cmd(0xA4);
    ssd1306_cmd(0xA6);

    ssd1306_cmd(0xAF); // DISPLAY ON
}

void turn_on_all() {
    // Turn on all LEDs
    set_LED(FORWARD_CHANNEL, 511), set_LED(BACKWARD_CHANNEL, 511), set_LED(LEFT_CHANNEL, 511), set_LED(RIGHT_CHANNEL, 511), gpio_set_level(MODE_PIN, 1);
    printf("Wait for all LEDs to turn off before using the device\n");
}

void turn_off_all() {
    // Turn off all LEDs
    set_LED(FORWARD_CHANNEL, 0), set_LED(BACKWARD_CHANNEL, 0), set_LED(LEFT_CHANNEL, 0), set_LED(RIGHT_CHANNEL, 0), gpio_set_level(MODE_PIN, 0);
    printf("Device is now ready for use!\n");
}