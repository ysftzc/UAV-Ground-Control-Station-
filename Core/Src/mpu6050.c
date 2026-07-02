/* MPU6050 I2C surucusu - GY-87 modulu icindeki IMU */
#include "mpu6050.h"

static float s_gyro_bias_x = 0.0f, s_gyro_bias_y = 0.0f, s_gyro_bias_z = 0.0f;

static HAL_StatusTypeDef MPU6050_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(hi2c, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0;

    /* Cihaz kimligini dogrula */
    status = MPU6050_ReadRegs(hi2c, MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    if (status != HAL_OK || who_am_i != MPU6050_WHO_AM_I_VAL) {
        return HAL_ERROR;
    }

    /* Sleep modundan cik, clock kaynagi olarak gyro X sec (PLL) */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_PWR_MGMT_1, 0x01);
    if (status != HAL_OK) return status;

    /* Sample rate = 1kHz / (1 + 7) = 125Hz */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_SMPLRT_DIV, 0x07);
    if (status != HAL_OK) return status;

    /* Digital low-pass filter: ~44Hz bant genisligi */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_CONFIG, 0x03);
    if (status != HAL_OK) return status;

    /* Gyro full-scale: +-250 deg/s */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (status != HAL_OK) return status;

    /* Accel full-scale: +-2g */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (status != HAL_OK) return status;

    /* I2C master modunu kapat, bypass'i ac - GY-87'de HMC5883L bu sayede
     * ana I2C hattinda dogrudan gorunur hale gelir (MPU6050'nin auxiliary
     * bus'inin arkasinda oldugu icin bypass olmadan hic yanit vermiyor) */
    status = MPU6050_WriteReg(hi2c, MPU6050_REG_USER_CTRL, 0x00);
    if (status != HAL_OK) return status;

    status = MPU6050_WriteReg(hi2c, MPU6050_REG_INT_PIN_CFG, 0x02);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data) {
    uint8_t raw[14];
    HAL_StatusTypeDef status;

    /* ACCEL_XOUT_H'dan itibaren 14 byte: accel(6) + temp(2) + gyro(6) */
    status = MPU6050_ReadRegs(hi2c, MPU6050_REG_ACCEL_XOUT_H, raw, sizeof(raw));
    if (status != HAL_OK) {
        return status;
    }

    int16_t accel_x_raw = (int16_t)((raw[0]  << 8) | raw[1]);
    int16_t accel_y_raw = (int16_t)((raw[2]  << 8) | raw[3]);
    int16_t accel_z_raw = (int16_t)((raw[4]  << 8) | raw[5]);
    /* raw[6..7] = sicaklik, kullanilmiyor */
    int16_t gyro_x_raw  = (int16_t)((raw[8]  << 8) | raw[9]);
    int16_t gyro_y_raw  = (int16_t)((raw[10] << 8) | raw[11]);
    int16_t gyro_z_raw  = (int16_t)((raw[12] << 8) | raw[13]);

    data->accel_x = accel_x_raw / MPU6050_ACCEL_LSB_PER_G;
    data->accel_y = accel_y_raw / MPU6050_ACCEL_LSB_PER_G;
    data->accel_z = accel_z_raw / MPU6050_ACCEL_LSB_PER_G;

    data->gyro_x = gyro_x_raw / MPU6050_GYRO_LSB_PER_DPS - s_gyro_bias_x;
    data->gyro_y = gyro_y_raw / MPU6050_GYRO_LSB_PER_DPS - s_gyro_bias_y;
    data->gyro_z = gyro_z_raw / MPU6050_GYRO_LSB_PER_DPS - s_gyro_bias_z;

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_CalibrateGyro(I2C_HandleTypeDef *hi2c, uint16_t sample_count) {
    if (sample_count == 0) {
        return HAL_ERROR;
    }

    double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
    uint16_t good = 0;
    MPU6050_Data_t sample;

    /* Bias henuz sifir oldugu icin MPU6050_ReadData burada ham degeri dondurur */
    for (uint16_t i = 0; i < sample_count; i++) {
        if (MPU6050_ReadData(hi2c, &sample) == HAL_OK) {
            sum_x += sample.gyro_x;
            sum_y += sample.gyro_y;
            sum_z += sample.gyro_z;
            good++;
        }
        HAL_Delay(5);
    }

    if (good == 0) {
        return HAL_ERROR;
    }

    s_gyro_bias_x = (float)(sum_x / good);
    s_gyro_bias_y = (float)(sum_y / good);
    s_gyro_bias_z = (float)(sum_z / good);

    return HAL_OK;
}

void MPU6050_GetGyroBias(float *bias_x, float *bias_y, float *bias_z) {
    *bias_x = s_gyro_bias_x;
    *bias_y = s_gyro_bias_y;
    *bias_z = s_gyro_bias_z;
}
