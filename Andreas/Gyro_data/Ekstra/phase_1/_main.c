#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_PIN 23
#define RGB_RED 22
#define RGB_GREEN 21
#define RGB_BLUE 19

void rgb_light(int color){
    int color_dict[8][3] = {
    {0,0,0}, // OFF
    {0,0,1}, // BLUE
    {0,1,0}, // GREEN
    {0,1,1}, // CYAN
    {1,0,0}, // RED
    {1,0,1}, // PURPLE
    {1,1,0}, // YELLOW
    {1,1,1}}; // WHITE
    gpio_set_level(RGB_RED, !color_dict[color][0]);
    gpio_set_level(RGB_GREEN, !color_dict[color][1]);
    gpio_set_level(RGB_BLUE, !color_dict[color][2]);
}

void start_up(){
  rgb_light(0);
  for(int i = 0; i<=2; i++){
    gpio_set_level(LED_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(LED_PIN, 0);
    if(i != 2) vTaskDelay(pdMS_TO_TICKS(1000));
  }
  for(int i = 0; i <= 7; i++){
    if (i == 0 || i == 7) continue;
    rgb_light(i);
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
  rgb_light(0);
}


void app_main() {
  gpio_config_t io_conf_outs = {
      .pin_bit_mask = (1ULL << LED_PIN) +
                      (1ULL << RGB_RED) +
                      (1ULL << RGB_GREEN) +
                      (1ULL << RGB_BLUE),
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
  };
  gpio_config(&io_conf_outs);
  start_up();
}
