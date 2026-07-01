/* BMP180 I2C surucusu - GY-87 modulu icindeki barometre */
#include "bmp180.h"
#include <math.h>

/* Fabrika kalibrasyon katsayilari - Init() sirasinda EEPROM'dan okunup saklanir */
static struct {
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2;
    int16_t  MB, MC, MD;
} s_calib;

static HAL_StatusTypeDef BMP180_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t value) {
    return HAL_I2C_Mem_Write(hi2c, BMP180_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

static HAL_StatusTypeDef BMP180_ReadRegs(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t *buf, uint16_t len) {
    return HAL_I2C_Mem_Read(hi2c, BMP180_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

HAL_StatusTypeDef BMP180_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t raw[22];
    HAL_StatusTypeDef status;

    /* Kalibrasyon EEPROM'unu tek seferde oku (0xAA..0xBF, 22 byte) */
    status = BMP180_ReadRegs(hi2c, BMP180_REG_CALIB_START, raw, sizeof(raw));
    if (status != HAL_OK) {
        return status;
    }

    s_calib.AC1 = (int16_t)((raw[0]  << 8) | raw[1]);
    s_calib.AC2 = (int16_t)((raw[2]  << 8) | raw[3]);
    s_calib.AC3 = (int16_t)((raw[4]  << 8) | raw[5]);
    s_calib.AC4 = (uint16_t)((raw[6]  << 8) | raw[7]);
    s_calib.AC5 = (uint16_t)((raw[8]  << 8) | raw[9]);
    s_calib.AC6 = (uint16_t)((raw[10] << 8) | raw[11]);
    s_calib.B1  = (int16_t)((raw[12] << 8) | raw[13]);
    s_calib.B2  = (int16_t)((raw[14] << 8) | raw[15]);
    s_calib.MB  = (int16_t)((raw[16] << 8) | raw[17]);
    s_calib.MC  = (int16_t)((raw[18] << 8) | raw[19]);
    s_calib.MD  = (int16_t)((raw[20] << 8) | raw[21]);

    /* Bos/arizali EEPROM tespiti - AC1 hicbir zaman 0 veya 0xFFFF olamaz (datasheet) */
    if (s_calib.AC1 == 0 || s_calib.AC1 == (int16_t)0xFFFF) {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef BMP180_ReadData(I2C_HandleTypeDef *hi2c, BMP180_Data_t *data) {
    uint8_t buf[3];
    HAL_StatusTypeDef status;

    /* Ham sicaklik olcumu */
    status = BMP180_WriteReg(hi2c, BMP180_REG_CONTROL, BMP180_CMD_TEMP);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    status = BMP180_ReadRegs(hi2c, BMP180_REG_RESULT_MSB, buf, 2);
    if (status != HAL_OK) return status;
    int32_t UT = (int32_t)((buf[0] << 8) | buf[1]);

    /* Ham basinc olcumu (OSS=0) */
    status = BMP180_WriteReg(hi2c, BMP180_REG_CONTROL, BMP180_CMD_PRESSURE_OSS0);
    if (status != HAL_OK) return status;
    HAL_Delay(5);

    status = BMP180_ReadRegs(hi2c, BMP180_REG_RESULT_MSB, buf, 3);
    if (status != HAL_OK) return status;
    int32_t UP = (int32_t)(((buf[0] << 16) | (buf[1] << 8) | buf[2]) >> (8 - BMP180_OSS));

    /* Bosch BMP180 datasheet - gercek sicaklik/basinc hesaplama algoritmasi */
    int32_t X1 = ((UT - (int32_t)s_calib.AC6) * (int32_t)s_calib.AC5) >> 15;
    int32_t X2 = ((int32_t)s_calib.MC << 11) / (X1 + s_calib.MD);
    int32_t B5 = X1 + X2;
    int32_t T  = (B5 + 8) >> 4;  /* 0.1 C birimi */

    int32_t B6 = B5 - 4000;
    X1 = ((int32_t)s_calib.B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)s_calib.AC2 * B6) >> 11;
    int32_t X3 = X1 + X2;
    int32_t B3 = ((((int32_t)s_calib.AC1 * 4 + X3) << BMP180_OSS) + 2) >> 2;

    X1 = ((int32_t)s_calib.AC3 * B6) >> 13;
    X2 = ((int32_t)s_calib.B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = (X1 + X2 + 2) >> 2;
    uint32_t B4 = (uint32_t)s_calib.AC4 * (uint32_t)(X3 + 32768) >> 15;
    uint32_t B7 = ((uint32_t)UP - (uint32_t)B3) * (50000 >> BMP180_OSS);

    int32_t p;
    if (B7 < 0x80000000UL) {
        p = (int32_t)((B7 << 1) / B4);
    } else {
        p = (int32_t)((B7 / B4) << 1);
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p = p + ((X1 + X2 + 3791) >> 4);  /* Pa */

    data->temperature = T / 10.0f;
    data->pressure     = p / 100.0f;  /* Pa -> hPa */
    data->altitude      = 44330.0f * (1.0f - powf(data->pressure / BMP180_SEA_LEVEL_HPA, 1.0f / 5.255f));

    return HAL_OK;
}
