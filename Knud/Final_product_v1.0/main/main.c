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

#define BUTTON_ROTATION    GPIO_NUM_10
#define BUTTON_CLEAR    GPIO_NUM_3
#define BUTTON_DRAW    GPIO_NUM_2 // Might want to change
#define MODE_PIN        GPIO_NUM_19

#define LEDC_MODE       LEDC_LOW_SPEED_MODE

#define FORWARD_CHANNEL     0
#define BACKWARD_CHANNEL    1
#define LEFT_CHANNEL        2
#define RIGHT_CHANNEL       3

#define RGB_TIMER       0
#define RGB_DUTY_RES    9       // 9-bit resolution
#define RGB_FREQUENCY   5000    // 5 kHz PWM
#define AVG_WINDOW      5

#define LED_OFF         0

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

typedef struct {float pos, vel, acc;} Axis_state;
typedef struct {int x, y;} Point;
typedef struct {
    Axis_state x, y, z;
    Point last_cell;
    uint8_t oled_buffer[128 * (64 / 8)];
} Gyro_state;

bool rot_led_level = false;
bool rotation_acceleration_swtich = true;
volatile bool clear_screen = false;
// volatile bool switch_active = false; // For increased drawing profiles
volatile bool draw_erase = true;
uint64_t count;
gptimer_handle_t gptimer = NULL;


// Prototype functions
void config_button(), config_i2c_master(), config_timer(), config_mode_led(), config_direction_channels(), config_led_timer(), config_ssd1306();

void init_acc_pos(acc_pos *f);

void acc_moving_avg_update(acc_pos *f, float ax, float ay, float az, float *ax_out, float *ay_out, float *az_out);

void mpu_write_reg(uint8_t reg, uint8_t val), mpu_read_reg(uint8_t reg, uint8_t *data, size_t len);

int16_t be16(const uint8_t *p);

int mapping(float x, float in_min, float in_max, int out_min, int out_max);

void calibration(float *ax_ptr, float *ay_ptr, float *az_ptr, float *temp_ptr, float *gx_ptr, float *gy_ptr, float *gz_ptr);

void interrupt_handler_1(), interrupt_handler_2(), interrupt_handler_3();

void set_LED(int CHANNEL, int value);

void ssd1306_cmd(uint8_t cmd);

void oled_update(Gyro_state *gs);

static inline void drive_axis(float value, float max_value, int neg_channel, int pos_channel);

void turn_on_all(), turn_off_all();

void init_Axis(Axis_state *axis);
void init_Gyro(Gyro_state *gs);
void step_Axis(Axis_state *axis, float acc_new);
void setPixel(Gyro_state *gs, bool on);


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

    // TODO: Startup sequence on display

    // Calibration
    printf("-- Beginning calibration --\n");
    float ax_cal = 0.0, ay_cal = 0.0, az_cal = 0.0, temp_cal = 0.0, gx_cal = 0.0, gy_cal = 0.0, gz_cal = 0.0;
    calibration(&ax_cal, &ay_cal, &az_cal, &temp_cal, &gx_cal, &gy_cal, &gz_cal);

    printf("-- Calibration complete-- \nAverage values:\n");
    printf("ax_cal = %.2f, ay_cal = %.2f, az_cal = %.2f, temp_cal = %.2f, gx_cal = %.2f, gy_cal = %.2f, gz_cal = %.2f\n\n",
        ax_cal, ay_cal, az_cal, temp_cal, gx_cal, gy_cal, gz_cal);

    turn_off_all(); // Startup ends

    // Running average of acceleration
    acc_pos acc_filter;
    init_acc_pos(&acc_filter);

    // Initialize gyro states (profiles) 
    Gyro_state g[1];
    for(int i = 0; i < 1; i++){
        init_Gyro(&g[i]);
    }

    int active = 0;


    while (1) {
        if (clear_screen) {
            init_Gyro(&g[active]);
            clear_screen = !clear_screen;
        }

        // // Swtiching drawing profile. Only in case more ram is available
        // if (switch_active) {
        //     switch_active = !switch_active;
        //     active += 1;
        //     active %= 1;
        //     setPixel(&g[active], draw_erase);
        // }

        gpio_set_level(MODE_PIN, rot_led_level);

        // Read data from the gyroscope
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

        // Temperature: (temp/340) + 36.53 (from datasheet)
        float temp_c = (temp / 340.0f) + 36.53f; // set - temp_cal for current temperature deviation instead of reading

        // Switching LEDs for different modes
        if (rotation_acceleration_swtich) {
            // In acceleration mode
            drive_axis(ay_g, 1.0f, BACKWARD_CHANNEL, FORWARD_CHANNEL);
            drive_axis(ax_g, 1.0f, LEFT_CHANNEL, RIGHT_CHANNEL);
        } else {
            // In rotation mode
            drive_axis(gy_dps, 250.0f, LEFT_CHANNEL, RIGHT_CHANNEL);
            drive_axis(gx_dps, 250.0f, FORWARD_CHANNEL, BACKWARD_CHANNEL);
        }

        // // Debug things
        // ESP_LOGI("IMU",
        //     "A[g]=(%5.2f,%5.2f,%5.2f) G[dps]=(%7.2f,%7.2f,%7.2f) T=%5.2fC",
        //     ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, temp_c);

        // Take steps on an axis
        step_Axis(&g[active].x, ay_g);
        step_Axis(&g[active].y, ax_g);

        printf(
            "%d, a:(%6.3f %6.3f %6.3f)\t"
            "v:(%6.3f %6.3f %6.3f)\t"
            "p:(%6.3f %6.3f %6.3f)\n",
            active,
            g[active].x.acc, g[active].y.acc, g[active].z.acc,
            g[active].x.vel, g[active].y.vel, g[active].z.vel,
            g[active].x.pos, g[active].y.pos, g[active].z.pos
        );

        // If drawing is false, erase pixels in an increased grid (code from the setPixel function)
        if (!draw_erase) {
            // Draw the pixels of the area to be deleted. 3x3 grid instead of just the one pixel
            for (int i = (int)g[active].x.pos - 1; i < g[active].x.pos + 2; i++) {
                for (int j = (int)g[active].y.pos - 1; j < g[active].y.pos + 2; j++) {
                    int x_int = i;
                    int x_mod = (x_int % OLED_WIDTH + OLED_WIDTH) % OLED_WIDTH;
                    int y_int = j;
                    int y_mod = (y_int % OLED_HEIGHT + OLED_HEIGHT) % OLED_HEIGHT;

                    uint8_t page = y_mod / 8;                           // which page
                    uint8_t bit  = y_mod % 8;                           // which bit
                    uint16_t index = index = page * OLED_WIDTH + x_mod; // byte in buffer
                    g[active].oled_buffer[index] |=  (1 << bit);        // set pixels
                }
            }
            oled_update(&g[active]);
            vTaskDelay(pdMS_TO_TICKS(5));

            // Delete the drawn pixels
            for (int i = (int)g[active].x.pos - 1; i < g[active].x.pos + 2; i++) {
                for (int j = (int)g[active].y.pos - 1; j < g[active].y.pos + 2; j++) {
                    int x_int = i;
                    int x_mod = (x_int % OLED_WIDTH + OLED_WIDTH) % OLED_WIDTH;
                    int y_int = j;
                    int y_mod = (y_int % OLED_HEIGHT + OLED_HEIGHT) % OLED_HEIGHT;

                    uint8_t page = y_mod / 8;                           // which page
                    uint8_t bit  = y_mod % 8;                           // which bit
                    uint16_t index = index = page * OLED_WIDTH + x_mod; // byte in buffer
                    g[active].oled_buffer[index] &= ~(1 << bit);        // erase pixels
                }
            }
            oled_update(&g[active]);
        } else {
            // If drawing, just set the active pixel
            setPixel(&g[active], draw_erase);
            oled_update(&g[active]);
        }
    }
}


// Functions

// Calibration of gyroscope. Creates average errors which are subtracted from gyroscope readings
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

        ax   += be16(&buf[0])  / 16384.0f;
        ay   += be16(&buf[2])  / 16384.0f;
        az   += be16(&buf[4])  / 16384.0f;
        temp += be16(&buf[6])  / 340.0f + 36.53f;
        gx   += be16(&buf[8])  / 131.0f;
        gy   += be16(&buf[10]) / 131.0f;
        gz   += be16(&buf[12]) / 131.0f;

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    *ax_ptr     = ax   / (float)n_steps;
    *ay_ptr     = ay   / (float)n_steps;
    *az_ptr     = az   / (float)n_steps;
    *temp_ptr   = temp / (float)n_steps;
    *gx_ptr     = gx   / (float)n_steps;
    *gy_ptr     = gy   / (float)n_steps;
    *gz_ptr     = gz   / (float)n_steps;
}

// Initialize running average
void init_acc_pos(acc_pos *f) {
    memset(f, 0, sizeof(acc_pos));
}

// Configure buttons and interrupt handlers
void config_button() {
    gpio_config_t io_conf_ins = {
        .pin_bit_mask = (1ULL << BUTTON_ROTATION) | (1ULL << BUTTON_CLEAR) | (1ULL << BUTTON_DRAW),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf_ins);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_ROTATION, interrupt_handler_1, (void *)BUTTON_ROTATION);
    gpio_isr_handler_add(BUTTON_CLEAR, interrupt_handler_2, (void *)BUTTON_CLEAR);
    gpio_isr_handler_add(BUTTON_DRAW, interrupt_handler_3, (void *)BUTTON_DRAW);
}

// Mode LED for rotation and acceleration mode
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
// Set ESP to master and what pins are data and clock
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

// Configure timer for interrupts
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
// Configureations led channels used for PWM
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
// Configureation of LED timer used for PWM
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
// Write value at register destination to the MPU6050
void mpu_write_reg(uint8_t reg, uint8_t val) {
  i2c_master_write_to_device(I2C_PORT, MPU_ADDR, (uint8_t[]){reg, val}, 2, pdMS_TO_TICKS(100));
}
// Read from register and put it into the data with size len
void mpu_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  i2c_master_write_read_device(I2C_PORT, MPU_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}
// Get values in big endian format
int16_t be16(const uint8_t *p) { 
    return (int16_t)((p[0] << 8) | p[1]);
}

// Maps some input x with an input min and max to an output min and max range 
int mapping(float x, float in_min, float in_max, int out_min, int out_max){
    return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

// Interrupt for switching mode
void interrupt_handler_1() {
    uint64_t new_count;
    gptimer_get_raw_count(gptimer, &new_count);
    if ((new_count - count) > 500000) {
        rot_led_level = !rot_led_level;
        rotation_acceleration_swtich = !rotation_acceleration_swtich;
        count = new_count;
    }
}

// Interrupt for clearing the screen
void interrupt_handler_2() {
    uint64_t new_count;
    gptimer_get_raw_count(gptimer, &new_count);
    if ((new_count - count) > 500000) {
        clear_screen = !clear_screen;
        count = new_count;
    }
}

// Interrupt for drawing and erasing
void interrupt_handler_3() {
    uint64_t new_count;
    gptimer_get_raw_count(gptimer, &new_count);
    if ((new_count - count) > 500000) {
        draw_erase = !draw_erase;
        count = new_count;
    }
}

// Set an LED with a given channel to a given value
void set_LED(int CHANNEL, int value) {
    ledc_set_duty(LEDC_MODE, CHANNEL, value);
    ledc_update_duty(LEDC_MODE, CHANNEL);
}

// Calculate running average
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

// Write to display
void ssd1306_cmd(uint8_t cmd) {
    i2c_cmd_handle_t h = i2c_cmd_link_create();
    i2c_master_start(h);
    i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(h, 0x00, true); // Control byte: command
    i2c_master_write_byte(h, cmd, true);
    i2c_master_stop(h);
    i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(h);
}

// Push buffer to display
void oled_update(Gyro_state *gs) {
    for (uint8_t page = 0; page < 8; page++) {
        ssd1306_cmd(0xB0 + page);  // select
        ssd1306_cmd(0x00);         // column low
        ssd1306_cmd(0x10);         // column high

        i2c_cmd_handle_t h = i2c_cmd_link_create();
        i2c_master_start(h);
        i2c_master_write_byte(h, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(h, 0x40, true); // data mode

        for (uint8_t x = 0; x < OLED_WIDTH; x++) {
            i2c_master_write_byte(
                h,
                gs->oled_buffer[page * OLED_WIDTH + x],
                true
            );
        }

        i2c_master_stop(h);
        ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_PORT, h, pdMS_TO_TICKS(100)));
        i2c_cmd_link_delete(h);
    }
}

// Initializing the OLED display
void config_ssd1306() {
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

// Turn on all LEDs
void turn_on_all() {
    set_LED(FORWARD_CHANNEL, 511), set_LED(BACKWARD_CHANNEL, 511), set_LED(LEFT_CHANNEL, 511), set_LED(RIGHT_CHANNEL, 511), gpio_set_level(MODE_PIN, 1);
    printf("Wait for all LEDs to turn off before using the device\n");
}

// Turn off all LEDs
void turn_off_all() {
    set_LED(FORWARD_CHANNEL, 0), set_LED(BACKWARD_CHANNEL, 0), set_LED(LEFT_CHANNEL, 0), set_LED(RIGHT_CHANNEL, 0), gpio_set_level(MODE_PIN, 0);
    printf("Device is now ready for use!\n");
}

static inline void drive_axis(float value, float max_value, int neg_channel, int pos_channel) {
    float mag = fabsf(value);
    if (mag > max_value) mag = max_value;

    int pwm = mapping(mag, 0.0f, max_value, 0, 511);

    if (value < 0) {
        set_LED(neg_channel, pwm);
        set_LED(pos_channel, LED_OFF);
    } else {
        set_LED(neg_channel, LED_OFF);
        set_LED(pos_channel, pwm);
    }
}

void init_Axis(Axis_state *axis) {axis->pos = axis->vel = axis->acc = .0f;}

void init_Gyro(Gyro_state *gs){
    init_Axis(&gs->x); init_Axis(&gs->y); init_Axis(&gs->z);
    gs->x.pos = (int)(OLED_WIDTH / 2);
    gs->y.pos = (int)(OLED_HEIGHT / 2);
    memset(gs->oled_buffer, 0, sizeof(gs->oled_buffer));
    gs->last_cell = (Point){(int)gs->x.pos, (int)gs->y.pos};
}

void step_Axis(Axis_state *axis, float acc_new){
    float dt = 0.5f;
    float vel_old = axis->vel * 0.95f;
    axis->vel = vel_old + 0.5f * dt * (acc_new + axis->acc);
    axis->pos = axis->pos + dt * vel_old + 0.25f * dt * dt * (axis->acc + acc_new);
    axis->acc = acc_new;
}

void setPixel(Gyro_state *gs, bool on){
    int x_int = (int) gs->x.pos;
    int x_mod = (x_int % OLED_WIDTH + OLED_WIDTH) % OLED_WIDTH;
    int y_int = (int) gs->y.pos;
    int y_mod = (y_int % OLED_HEIGHT + OLED_HEIGHT) % OLED_HEIGHT;

    uint8_t page = y_mod / 8;                           // which page
    uint8_t bit  = y_mod % 8;                           // which bit
    uint16_t index = index = page * OLED_WIDTH + x_mod; // byte in buffer

    if(on){
        gs->oled_buffer[index] |=  (1 << bit);
    } else
        gs->oled_buffer[index] &= ~(1 << bit);
}