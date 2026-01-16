#ifndef PRE_DEFINE
#define PRE_DEFINE


#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     8
#define I2C_SCL_PIN     9
#define I2C_FREQ_HZ     400000

#define MPU_ADDR        0x68

// MPU6050 registre
#define REG_WHO_AM_I    0x75
#define REG_PWR_MGMT_1  0x6B
#define REG_ACCEL_XOUT  0x3B  // start for accel/gyro burst read


#endif