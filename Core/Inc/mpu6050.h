/* MPU6050 I2C surucusu - GY-87 modulu icindeki IMU */
#ifndef __MPU6050_H__
#define __MPU6050_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

/* I2C adresi - AD0 pin GND'e bagli, HAL 8-bit (shifted) adres bekliyor */
#define MPU6050_I2C_ADDR        (0x68 << 1)

/* Register adresleri */
#define MPU6050_REG_SMPLRT_DIV  0x19
#define MPU6050_REG_CONFIG      0x1A
#define MPU6050_REG_GYRO_CONFIG 0x1B
#define MPU6050_REG_ACCEL_CONFIG 0x1C
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_PWR_MGMT_1  0x6B
#define MPU6050_REG_WHO_AM_I    0x75
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_USER_CTRL   0x6A

#define MPU6050_WHO_AM_I_VAL    0x68

/* Full-scale hassasiyet bolucileri (FS_SEL=0, AFS_SEL=0 icin) */
#define MPU6050_ACCEL_LSB_PER_G     16384.0f  /* +-2g */
#define MPU6050_GYRO_LSB_PER_DPS    131.0f    /* +-250 deg/s */

typedef struct {
    float accel_x, accel_y, accel_z;  /* g */
    float gyro_x, gyro_y, gyro_z;     /* derece/saniye */
} MPU6050_Data_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __MPU6050_H__ */
