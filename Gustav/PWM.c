#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define RGB_RED GPIO_NUM_1
#define RGB_GREEN GPIO_NUM_0
#define RGB_BLUE GPIO_NUM_3                
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define RED_CHANNEL    0
#define GREEN_CHANNEL    1
#define BLUE_CHANNEL    2
#define RGB_TIMER      0
#define RGB_DUTY_RES   13                  // 13-bit resolution
#define RGB_FREQUENCY  5000                // 5 kHz PWM

void app_main(void)
{
    // Configure RGB timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = RGB_DUTY_RES,
        .timer_num        = RGB_TIMER,
        .freq_hz          = RGB_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // Configure RED RGB channel
    ledc_channel_config_t red_channel_cfg = {
        .gpio_num       = RGB_RED,
        .speed_mode     = LEDC_MODE,
        .channel        = RED_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&red_channel_cfg);

    // Configure GREEN RGB channel
    ledc_channel_config_t green_channel_cfg = {
        .gpio_num       = RGB_GREEN,
        .speed_mode     = LEDC_MODE,
        .channel        = GREEN_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&green_channel_cfg);

    // Configure BLUE RGB channel
    ledc_channel_config_t blue_channel_cfg = {
        .gpio_num       = RGB_BLUE,
        .speed_mode     = LEDC_MODE,
        .channel        = BLUE_CHANNEL,
        .timer_sel      = RGB_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&blue_channel_cfg);

    //Turn off RGB - RBG IS INVERTED: 8191 = OFF, 0 = ON
    // RED
    ledc_set_duty(LEDC_MODE, RED_CHANNEL, 8191);
    ledc_update_duty(LEDC_MODE, RED_CHANNEL);
    // GREEN
    ledc_set_duty(LEDC_MODE, GREEN_CHANNEL, 8191);
    ledc_update_duty(LEDC_MODE, GREEN_CHANNEL);
    // BLUE
    ledc_set_duty(LEDC_MODE, BLUE_CHANNEL, 8191);
    ledc_update_duty(LEDC_MODE, BLUE_CHANNEL);
    vTaskDelay(pdMS_TO_TICKS(5000));

  
    int randomNumbers[9] = {8000, 8000, 8000, 0, 0, 8000, 8000, 8000, 8000};
    while (true) {
        // RGB_RED settings
        for (int i = 0; i <= 8; i++) {
            printf("RED %d\n", i);
            ledc_set_duty(LEDC_MODE, RED_CHANNEL, randomNumbers[i]);
            ledc_update_duty(LEDC_MODE, RED_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(1000));    
        }
        // RGB_GREEN settings
        for (int i = 0; i <= 8; i++) {
            printf("GREEN %d\n", i);
            ledc_set_duty(LEDC_MODE, GREEN_CHANNEL, randomNumbers[i]);
            ledc_update_duty(LEDC_MODE, GREEN_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        // RGB_BLUE settings
        for (int i = 0; i <= 8; i++) {
           printf("BLUE %d\n", i);
            ledc_set_duty(LEDC_MODE, BLUE_CHANNEL, randomNumbers[i]);
            ledc_update_duty(LEDC_MODE, BLUE_CHANNEL);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}