#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "soc/gpio_reg.h"
#include "esp_adc/adc_oneshot.h"


void app_main(void)
{   // Konfigurerer den analoge sensor på pin 2 på microcontrolleren
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

    
    // Printer den læste value fra photocell hvert halve sekund.
    // Lav value = mørkt, høj value = lyst.
    while(1) {
        int adc_raw;
        adc_oneshot_read(adc_handle, ADC_CHANNEL_2, &adc_raw);
        printf("Value measured: %d\n", adc_raw);
        vTaskDelay(pdMS_TO_TICKS(500));;
    }
}
    