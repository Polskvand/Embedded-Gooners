#include <INITIALISER_PHASE2.h>


void init_acc_pos(acc_pos *f){
    memset(f, 0, sizeof(acc_pos));
}

void gpio_configure(){
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

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MODE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io_conf);

    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1 * 1000 * 1000,
    };
    gptimer_new_timer(&timer_config, &gptimer);
    gptimer_enable(gptimer);
    gptimer_start(gptimer);

    gpio_config_t io_conf_ins = {
        .pin_bit_mask = (1ULL << BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&io_conf_ins);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_PIN, interrupt_handler, NULL);
    gptimer_get_raw_count(gptimer, &count);
    mpu_write_reg(REG_PWR_MGMT_1, 0x00);

    // Configure RGB timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = RGB_DUTY_RES,
        .timer_num        = RGB_TIMER,
        .freq_hz          = RGB_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    
    // Configure direction channels
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

