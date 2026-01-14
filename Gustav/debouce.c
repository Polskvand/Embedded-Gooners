#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/gpio_reg.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "esp_rom_sys.h"


#define LED_PIN1 15
#define LED_PIN2 2
#define LED_PIN3 0
#define LED_PIN4 4
#define LED_PIN5 16
#define LED_PIN6 17
#define LED_PIN7 5
#define LED_PIN8 18
#define BUTTON_PIN 26

void interrupt_handler();

int direction = 0;
uint64_t count;

  gptimer_handle_t gptimer = NULL;

int LEDS[] = {LED_PIN1, LED_PIN2, LED_PIN3, LED_PIN4, LED_PIN5, LED_PIN6, LED_PIN7, LED_PIN8};
int i = 0;

void app_main(void)
{
    gpio_config_t io_conf_outs = {
      .pin_bit_mask = (1ULL << LED_PIN1) + (1ULL << LED_PIN2) + (1ULL << LED_PIN3) + (1ULL << LED_PIN4) + (1ULL << LED_PIN5) + (1ULL << LED_PIN6) + (1ULL << LED_PIN7) + (1ULL << LED_PIN8), 
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_outs);


  // Configure a timer
  gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1 * 1000 * 1000,   // Resolution is 1 MHz, i.e., 1 tick equals 1 microsecond
  };
  gptimer_new_timer(&timer_config, &gptimer);

  // Enable the timer
  gptimer_enable(gptimer);
  // Start the timer
  gptimer_start(gptimer);

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

  gptimer_get_raw_count(gptimer, &count);

  while (true) {
    for (; i < 7; i++) {
      if (direction) {
        direction = 0;
        break;
      }
      int pin_set = 1;
      gpio_set_level(LEDS[i], pin_set);
      pin_set = !pin_set;
      vTaskDelay(pdMS_TO_TICKS(400));;
      gpio_set_level(LEDS[i], pin_set);
      
      
    }

    for (; i > 0; i--) {
      if (direction) {
        direction = 0;
        break;
      }
      int pin_set = 1;
      gpio_set_level(LEDS[i], pin_set);
      pin_set = !pin_set;
      vTaskDelay(pdMS_TO_TICKS(400));;
      gpio_set_level(LEDS[i], pin_set);
    }
  }
    
}

void interrupt_handler()
{
  uint64_t new_count;
  gptimer_get_raw_count(gptimer, &new_count);
  
  if ((new_count - count) > 500000) {
    direction = !direction;
    count = new_count;
  }
  
}
