#ifndef PRE_DEFINE
#define PRE_DEFINE

#include "LIBRARIES_PHASE2.h"

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     GPIO_NUM_5
#define I2C_SCL_PIN     GPIO_NUM_6
#define I2C_FREQ_HZ     400000

#define MPU_ADDR        0x68
#define REG_PWR_MGMT_1     0x6B
#define REG_ACCEL_XOUT     0x3B
#define REG_ACCEL_CONFIG   0x1C

#define LED_FORWARD     GPIO_NUM_1
#define LED_BACKWARD    GPIO_NUM_9
#define LED_LEFT        GPIO_NUM_7
#define LED_RIGHT       GPIO_NUM_0

#define BUTTON_PIN      GPIO_NUM_4
#define MODE_PIN        GPIO_NUM_19

#define LEDC_MODE       LEDC_LOW_SPEED_MODE

#define FORWARD_CHANNEL     0
#define BACKWARD_CHANNEL    1
#define LEFT_CHANNEL        2
#define RIGHT_CHANNEL       3

#define RGB_TIMER       0
#define RGB_DUTY_RES    9
#define RGB_FREQUENCY   5000

#define PI          3.14159265359f
#define AVG_WINDOW  32

typedef struct {
    float ax_buf[AVG_WINDOW];
    float ay_buf[AVG_WINDOW];
    float az_buf[AVG_WINDOW];

    float ax_sum;
    float ay_sum;
    float az_sum;

    uint8_t index;
    bool filled;
} acc_pos;

extern volatile bool rot_led_level;
extern volatile bool rotation_acceleration_swtich;
extern uint64_t count;
extern gptimer_handle_t gptimer;

#endif