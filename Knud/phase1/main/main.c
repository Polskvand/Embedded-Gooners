#include <stdbool.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "soc/gpio_reg.h"
#include "esp_adc/adc_oneshot.h"


#define LED_PIN GPIO_NUM_9
#define BUTTON_PIN GPIO_NUM_4
#define SDA_PIN GPIO_NUM_5 
#define SCL_PIN GPIO_NUM_6
#define AM2320_ADDR 0x5C
#define I2C_PORT 0
#define RED_LED GPIO_NUM_0
#define GREEN_LED GPIO_NUM_1
#define BLUE_LED GPIO_NUM_3


volatile int led_level = 1;
volatile bool temp_humid_sensor = true;
int program_begun = false;
volatile int BUTTON_PRESSED = 0;

void red(), green(), blue(), purple(), yellow(), white(), cyan(), all_off();

void interrupt_handler()
{
    BUTTON_PRESSED = 1;
    if (program_begun) { // main while loop has begun
        led_level = !led_level;
        temp_humid_sensor = !temp_humid_sensor;
    }     
}


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

void app_main(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_PIN) + (1ULL << RED_LED) + (1ULL << GREEN_LED) + (1ULL << BLUE_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    all_off();

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


    // Konfigurerer den analoge sensor på pin 2 på microcontrolleren
    adc_oneshot_unit_handle_t adc_handle;

    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_cfg, &adc_handle);

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_11,
    };

    adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_2, &chan_cfg);

   
    // // Startup sequence
    // //Tænder og slukker for LED 3 gange
    // for (int i = 0; i < 3; i++){
    //     gpio_set_level(LED_PIN, 1);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     gpio_set_level(LED_PIN, 0);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    // }

    // //Cycle through RGB colours
    // red();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // green();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // blue();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // purple();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // yellow();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // white();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // cyan();
    // vTaskDelay(pdMS_TO_TICKS(1000));
    // all_off();
    





    // Light sensor calibration
    int low_value = -1;
    int high_value = -1;
    
    printf("Cover the thing and press the button\n");

    while (true) {
        if (low_value == -1 && BUTTON_PRESSED) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &low_value);
            BUTTON_PRESSED = 0;
            printf("thank you\n");
            vTaskDelay(pdMS_TO_TICKS(50));

            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    printf("Stop covering the thing and press the button\n");

    while (true) {
        if (high_value == -1 && BUTTON_PRESSED) {
            adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &high_value);
            BUTTON_PRESSED = 0;
            printf("thank you again\n");
            vTaskDelay(pdMS_TO_TICKS(50));

            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    printf("low value:  %d\nhigh value: %d\n", low_value, high_value);


    int new_max_value = high_value - low_value;
    int interval = new_max_value / 8; // divide by number of colours used




    vTaskDelay(pdMS_TO_TICKS(1000));

    program_begun = true;

    while (true) {
        gpio_set_level(LED_PIN, led_level);
        vTaskDelay(pdMS_TO_TICKS(10));

        if (temp_humid_sensor) {
            // Temperature and humidity readout

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

            // CRC check
            unsigned short crc_code = crc16(read, 6);
            unsigned short crc_calc = (read[7] << 8) + (read[6]);

            // printf("crc calc: 0x%04X\n", crc_code);
            // printf("crc revc: 0x%02X%02X\n", read[7], read[6]);

            if (crc_code == crc_calc) {
                printf("Temp: %.2f\n", (float)temp / 10);
                // printf("Humid: %.2f%% \n", (float)humid /10);

                // RGB
                if (temp < 230) {
                    // purple
                    gpio_set_level(RED_LED, 0);
                    gpio_set_level(GREEN_LED, 1);
                    gpio_set_level(BLUE_LED, 0);
                }
                else if (temp > 230 && temp < 240) {
                    // green
                    gpio_set_level(RED_LED, 1);
                    gpio_set_level(GREEN_LED, 0);
                    gpio_set_level(BLUE_LED, 1);
                }
                else if (temp > 240 && temp < 250) {
                    // red
                    gpio_set_level(RED_LED, 0);
                    gpio_set_level(GREEN_LED, 1);
                    gpio_set_level(BLUE_LED, 1);
                }
                else if (temp > 250) {
                    // white
                    gpio_set_level(RED_LED, 0);
                    gpio_set_level(GREEN_LED, 0);
                    gpio_set_level(BLUE_LED, 0);
                }
            }
        }
        

        if (!temp_humid_sensor) {
            // Light sensor readout

            // Printer den læste value fra photocell hvert halve sekund.
            // Lav value = mørkt, høj value = lyst.
            int adc_raw;
            adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &adc_raw);
            printf("Value measured: %d\n", adc_raw);
            int calibrated_measure = adc_raw - low_value;

            if (calibrated_measure < interval * 1) {
                all_off();
            } else if (calibrated_measure < interval * 2) {
                purple();
            } else if (calibrated_measure < interval * 3) {
                blue();
            } else if (calibrated_measure < interval * 4) {
                cyan();
            } else if (calibrated_measure < interval * 5) {
                green();
            } else if (calibrated_measure < interval * 6) {
                yellow();
            } else if (calibrated_measure < interval * 7) {
                red();
            } else if (calibrated_measure < interval * 8) {
                white();
            } 
        }
    }
}

void red() {
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 1);
    gpio_set_level(BLUE_LED, 1);
}

void green() {
    gpio_set_level(RED_LED, 1);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 1);
}

void blue() {
    gpio_set_level(RED_LED, 1);
    gpio_set_level(GREEN_LED, 1);
    gpio_set_level(BLUE_LED, 0);
}

void purple() {
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 1);
    gpio_set_level(BLUE_LED, 0);
}

void yellow() {
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 1);
}

void white() {
    gpio_set_level(RED_LED, 0);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 0);
}

void cyan() {
    gpio_set_level(RED_LED, 1);
    gpio_set_level(GREEN_LED, 0);
    gpio_set_level(BLUE_LED, 0);
};

void all_off() {
    gpio_set_level(RED_LED, 1);
    gpio_set_level(GREEN_LED, 1);
    gpio_set_level(BLUE_LED, 1);
};