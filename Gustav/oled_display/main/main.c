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

#define OLED_ADDR       0x3C
#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_5
#define I2C_SCL_PIN     GPIO_NUM_6
#define I2C_FREQ_HZ     400000

void i2c_master_init();
void oled_write_reg(uint8_t reg, uint8_t val);
void oled_read_reg(uint8_t reg, uint8_t *data, size_t len);
void oled_send_commands(const uint8_t *cmds, size_t len);
void oled_send_data(const uint8_t *data, size_t len);
void oled_set_position(uint8_t x, uint8_t page);
void oled_set_pixel(uint8_t x, uint8_t y);
void oled_fill(uint8_t value);

void app_main(void)
{
    i2c_master_init();

    // OLED display stuff
    static const uint8_t oled_init_cmds[] = {
        0xAE,       // Display OFF
        0xD5, 0x80, // Clock divide
        0xA8, 0x3F, // Multiplex 1/64
        0xD3, 0x00, // Display offset
        0x40,       // Start line
        0x8D, 0x14, // Charge pump ON
        0x20, 0x00, // Horizontal addressing
        0xA1,       // Segment remap
        0xC8,       // COM scan direction
        0xDA, 0x12, // COM pins config
        0x81, 0x7F, // Contrast
        0xD9, 0xF1, // Pre-charge
        0xDB, 0x40, // VCOM detect
        0xA4,       // Resume RAM display
        0xA6,       // Normal display
        0xAF        // Display ON
    };

    printf("OLED test\n");

    // uint8_t reset_cmd = 0xE2; // software reset (not always documented)
    // oled_send_commands(&reset_cmd, 1);
    // vTaskDelay(pdMS_TO_TICKS(50));

    oled_send_commands(oled_init_cmds, sizeof(oled_init_cmds));
    vTaskDelay(pdMS_TO_TICKS(100));

    uint8_t on = 0xAF;

    oled_fill(0xFF);   // Entire screen ON
    vTaskDelay(pdMS_TO_TICKS(2000));
    oled_fill(0x00);   // Entire screen OFF

    while (1)
    {
        oled_send_commands(&on,1);
        vTaskDelay(pdMS_TO_TICKS(1000));
        oled_set_pixel(10, 10);
        vTaskDelay(pdMS_TO_TICKS(1000));
        oled_set_pixel(20, 20);
        vTaskDelay(pdMS_TO_TICKS(1000));
        oled_set_pixel(30, 30);
        vTaskDelay(pdMS_TO_TICKS(1000));
        oled_set_pixel(40, 40);
        vTaskDelay(pdMS_TO_TICKS(1000));
        printf("In while loop\n");
    }
    
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

void oled_write_reg(uint8_t reg, uint8_t val) {
  i2c_master_write_to_device(I2C_PORT, OLED_ADDR, (uint8_t[]){reg, val}, 2, pdMS_TO_TICKS(100));
}

void oled_read_reg(uint8_t reg, uint8_t *data, size_t len) {
  i2c_master_write_read_device(I2C_PORT, OLED_ADDR, &reg, 1, data, len, pdMS_TO_TICKS(100));
}

void oled_send_commands(const uint8_t *cmds, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (OLED_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, 0x00, true);
    i2c_master_write(cmd, cmds, len, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
}

void oled_send_data(const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,
        (OLED_ADDR << 1) | I2C_MASTER_WRITE,
        true);

    // Control byte: DATA
    i2c_master_write_byte(cmd, 0x40, true);

    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
}

void oled_set_position(uint8_t x, uint8_t page)
{
    uint8_t cmds[] = {
        0x21, x, 127,   // Column address
        0x22, page, page // Page address
    };
    oled_send_commands(cmds, sizeof(cmds));
}

void oled_set_pixel(uint8_t x, uint8_t y)
{
    uint8_t page = y / 8;
    uint8_t bit  = y % 8;

    oled_set_position(x, page);

    uint8_t data = (1 << bit);
    oled_send_data(&data, 1);
}

void oled_fill(uint8_t value)
{
    for (uint8_t page = 0; page < 8; page++)
    {
        oled_set_position(0, page);

        uint8_t line[128];
        memset(line, value, sizeof(line));
        oled_send_data(line, sizeof(line));
    }
}

