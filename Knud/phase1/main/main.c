#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "soc/gpio_reg.h"


#define LED_PIN GPIO_NUM_9
#define BUTTON_PIN GPIO_NUM_1
#define SDA_PIN GPIO_NUM_5 
#define SCL_PIN GPIO_NUM_6
#define AM2320_ADDR 0x5C
#define I2C_PORT 0

volatile int led_level = 0;
volatile bool temp_humid_sensor = true;

void interrupt_handler()
{
    led_level = !led_level;
    temp_humid_sensor = !temp_humid_sensor;
}

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    // Configure Button
    gpio_config_t io_conf_ins = {
        .pin_bit_mask = (1ULL << BUTTON_PIN), 
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf_ins);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, interrupt_handler, NULL);

    // Configure i2c protocol
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000,
        .clk_flags = 0
    };

    i2c_param_config(I2C_NUM_0, &i2c_conf);
    i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0,0);

    //wake
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);


    //write
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    uint8_t req[4] =
    {
        (AM2320_ADDR << 1) | I2C_MASTER_WRITE,
        0X03,
        0X00,
        0X04
    };

    i2c_master_write(cmd, req, sizeof(req), true);

    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);

    while (true) {

        gpio_set_level(LED_PIN, led_level);
        vTaskDelay(pdMS_TO_TICKS(10));


        if (temp_humid_sensor) {
            // Temperature and humidity readout

            //read
            uint8_t read[8];

            vTaskDelay(pdMS_TO_TICKS(2));

            cmd = i2c_cmd_link_create();
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, (AM2320_ADDR << 1) | I2C_MASTER_READ, true);
            i2c_master_read(cmd, read, 8, false);
            i2c_master_stop(cmd);
            i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(50));
            i2c_cmd_link_delete(cmd);

            uint16_t temp =     (read[4] << 8) | read[5];
            uint16_t humid =    (read[2] << 8) | read[3];

            printf("Temp: %.2f \n", (float)temp / 10);
            printf("Humid: %.2f%% \n", (float)humid /10);

            // CRC check
            unsigned short crc16(unsigned char *ptr, unsigned char len)
            {
                unsigned short crc =0xFFFF;
                unsigned char i;

                while(len--)
                {
                    crc ^=*ptr++;
                    for(i=0;i<8;i++)
                    {
                        if(crc & 0x01)
                        {
                        crc>>=1;
                        crc^=0xA001;
                        }else
                        {
                        crc>>=1;
                        }
                    }
                }
                return crc;
            }

            printf("crc calc: 0x%04X\n", crc16(read, 6));
            printf("crc revc = 0x%02X%02X\n", read[7], read[6]);
        }
        

        if (!temp_humid_sensor) {
            // Light readout

        }
        
    }
}
