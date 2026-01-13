#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"

#define BLUE_LED         0
#define RED_LED         5
#define GREEN_LED        16                 
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL    0
#define LEDC_TIMER      0
#define LEDC_DUTY_RES   13                  // 13-bit resolution
#define LEDC_FREQUENCY  5000                // 5 kHz PWM

void app_main(void)
{
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    // // Configure RED RGB channel
    // ledc_channel_config_t red_channel_cfg = {
    //     .gpio_num       = RED_LED,
    //     .speed_mode     = LEDC_MODE,
    //     .channel        = 0,
    //     .timer_sel      = LEDC_TIMER,
    //     .duty           = 0,
    //     .hpoint         = 0
    // };
    // ledc_channel_config(&red_channel_cfg);

    // // Configure BLUE RGB channel
    // ledc_channel_config_t blue_channel_cfg = {
    //     .gpio_num       = BLUE_LED,
    //     .speed_mode     = LEDC_MODE,
    //     .channel        = 2,
    //     .timer_sel      = LEDC_TIMER,
    //     .duty           = 0,
    //     .hpoint         = 0
    // };
    // ledc_channel_config(&blue_channel_cfg);

    // Configure GREEN RGB channel
    ledc_channel_config_t green_channel_cfg = {
        .gpio_num       = GREEN_LED,
        .speed_mode     = LEDC_MODE,
        .channel        = 1,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&green_channel_cfg);

  
    int randomNumbers[8] = {8000, 6000, 4000, 50, 6000, 8000, 0, 8000};
    while (true) {
        //Fade up RED
        for (int i = 0; i <= 7; i++) {
            ledc_set_duty(LEDC_MODE, 1, randomNumbers[i]);
            ledc_update_duty(LEDC_MODE, 1);
            vTaskDelay(pdMS_TO_TICKS(2000));
            printf("i = %d\n", i);
        }
        // // Fade up GREEN
        // for (uint32_t duty = 0; duty <= max_duty; duty += step) {
        //     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        //     ledc_update_duty(LEDC_MODE, 1);
        //     vTaskDelay(pdMS_TO_TICKS(delay_ms));
        // }
        // // Fade up BLUE
        // for (uint32_t duty = 0; duty <= max_duty; duty += step) {
        //     ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        //     ledc_update_duty(LEDC_MODE, 2);
        //     vTaskDelay(pdMS_TO_TICKS(delay_ms));
        // }

        // // Fade down
        // for (int duty = max_duty; duty >= 0; duty -= step) {
        //     ledc_set_duty(LEDC_MODE, 1, duty);
        //     ledc_update_duty(LEDC_MODE, 1);
        //     vTaskDelay(pdMS_TO_TICKS(delay_ms));
        // }

        // // Turn off timer for 2 seconds
        // ledc_timer_pause(LEDC_MODE, LEDC_TIMER);
        // vTaskDelay(pdMS_TO_TICKS(2000));
        // ledc_timer_resume(LEDC_MODE, LEDC_TIMER);
    }
}