/* BMP180 I2C surucusu - GY-87 modulu icindeki barometre */
#ifndef __BMP180_H__
#define __BMP180_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

/* I2C adresi - HAL 8-bit (shifted) adres bekliyor */
#define BMP180_I2C_ADDR         (0x77 << 1)

/* Register adresleri */
#define BMP180_REG_CALIB_START  0xAA  /* 22 byte kalibrasyon EEPROM */
#define BMP180_REG_CONTROL      0xF4
#define BMP180_REG_RESULT_MSB   0xF6

/* Kontrol komutlari */
#define BMP180_CMD_TEMP         0x2E  /* sicaklik olcumu baslat, 4.5ms bekle */
#define BMP180_CMD_PRESSURE_OSS0 0x34 /* basinc olcumu baslat (OSS=0), 4.5ms bekle */

/* Oversampling ayari - basitlik icin en dusuk (OSS=0) kullanildi */
#define BMP180_OSS              0

#define BMP180_SEA_LEVEL_HPA    1013.25f

typedef struct {
    float pressure;     /* hPa */
    float temperature;  /* Celsius */
    float altitude;     /* metre (deniz seviyesi 1013.25 hPa varsayimiyla) */
} BMP180_Data_t;

HAL_StatusTypeDef BMP180_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef BMP180_ReadData(I2C_HandleTypeDef *hi2c, BMP180_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* __BMP180_H__ */
