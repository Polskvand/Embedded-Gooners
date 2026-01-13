#include <stdio.h>
#include "driver/gpio.h"

#define RGB_RED GPIO_NUM_1
#define RGB_GREEN GPIO_NUM_0
#define RGB_BLUE GPIO_NUM_3

main() {
    gpio_config_t io_conf_outs = {
        .pin_bit_mask = (1ULL << RGB_RED) + (1ULL << RGB_GREEN) + (1ULL << RGB_BLUE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf_outs);

}