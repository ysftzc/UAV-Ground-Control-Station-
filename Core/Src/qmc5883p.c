/* QMC5883P I2C surucusu - GY-87 modulu icindeki manyetometre (bkz. qmc5883p.h) */
#include "qmc5883p.h"

static float s_offset_x = 0.0f, s_offset_y = 0.0f, s_offset_z = 0.0f;

static HAL_StatusTypeDef QMC5883P_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, QMC5883P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef QMC5883P_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(hi2c, QMC5883P_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

HAL_StatusTypeDef QMC5883P_Init(I2C_HandleTypeDef *hi2c) {
    HAL_StatusTypeDef status;
    uint8_t reg_status;

    /* Soft reset */
    status = QMC5883P_WriteReg(hi2c, QMC5883P_REG_CTRL2, QMC5883P_CTRL2_SOFT_RESET);
    if (status != HAL_OK) return status;
    HAL_Delay(2);

    /* Eksen isareti tanimi - datasheet'te sabit deger olarak isteniyor */
    status = QMC5883P_WriteReg(hi2c, QMC5883P_REG_SIGN, 0x06);
    if (status != HAL_OK) return status;

    /* Olcum araligi: en hassas (2 Gauss, 15000 LSB/Gauss) */
    status = QMC5883P_WriteReg(hi2c, QMC5883P_REG_CTRL2, QMC5883P_CTRL2_RANGE_2G);
    if (status != HAL_OK) return status;

    /* Normal mod, 200Hz ODR, 8x oversample, 8x downsample */
    status = QMC5883P_WriteReg(hi2c, QMC5883P_REG_CTRL1, QMC5883P_CTRL1_CONFIG);
    if (status != HAL_OK) return status;

    /* Ilk olcumun hazir olmasini bekle (DRDY) - cihazin gercekten yanit
     * verip vermedigini de bu sayede dogrulamis oluyoruz (WHO_AM_I yok) */
    for (int i = 0; i < 100; i++) {
        HAL_Delay(1);
        status = QMC5883P_ReadRegs(hi2c, QMC5883P_REG_STATUS, &reg_status, 1);
        if (status == HAL_OK && (reg_status & 0x01)) {
            /* Range secimi bazi klonlarda mod ayarindan sonra sifirlanabiliyor, tekrar yaz */
            return QMC5883P_WriteReg(hi2c, QMC5883P_REG_CTRL2, QMC5883P_CTRL2_RANGE_2G);
        }
    }

    return HAL_ERROR;
}

HAL_StatusTypeDef QMC5883P_ReadData(I2C_HandleTypeDef *hi2c, QMC5883P_Data_t *data) {
    uint8_t raw[6];
    HAL_StatusTypeDef status;

    /* X_L,X_H,Y_L,Y_H,Z_L,Z_H - little-endian, HMC5883L'in aksine LSB once geliyor */
    status = QMC5883P_ReadRegs(hi2c, QMC5883P_REG_DATA_START, raw, sizeof(raw));
    if (status != HAL_OK) {
        return status;
    }

    int16_t mag_x_raw = (int16_t)((raw[1] << 8) | raw[0]);
    int16_t mag_y_raw = (int16_t)((raw[3] << 8) | raw[2]);
    int16_t mag_z_raw = (int16_t)((raw[5] << 8) | raw[4]);

    data->mag_x = (mag_x_raw / QMC5883P_LSB_PER_GAUSS) * QMC5883P_GAUSS_TO_UT - s_offset_x;
    data->mag_y = (mag_y_raw / QMC5883P_LSB_PER_GAUSS) * QMC5883P_GAUSS_TO_UT - s_offset_y;
    data->mag_z = (mag_z_raw / QMC5883P_LSB_PER_GAUSS) * QMC5883P_GAUSS_TO_UT - s_offset_z;

    return HAL_OK;
}

void QMC5883P_SetHardIronOffset(float offset_x, float offset_y, float offset_z) {
    s_offset_x = offset_x;
    s_offset_y = offset_y;
    s_offset_z = offset_z;
}

void QMC5883P_GetHardIronOffset(float *offset_x, float *offset_y, float *offset_z) {
    *offset_x = s_offset_x;
    *offset_y = s_offset_y;
    *offset_z = s_offset_z;
}
