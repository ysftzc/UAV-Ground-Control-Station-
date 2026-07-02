/* QMC5883P I2C surucusu - GY-87 modulu icindeki manyetometre.
 * NOT: Bu kart "HMC5883L" olarak satilsa da bus taramasinda 0x1E yerine
 * 0x2C'de yanit veren bir QST QMC5883P klonu tespit edildi - register
 * haritasi HMC5883L/QMC5883L ile tamamen farkli. Register/bit tanimlari
 * QST'nin resmi register haritasi referans alinarak dogrulandi. */
#ifndef __QMC5883P_H__
#define __QMC5883P_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "i2c.h"

/* I2C adresi - HAL 8-bit (shifted) adres bekliyor */
#define QMC5883P_I2C_ADDR       (0x2C << 1)

/* Register adresleri */
#define QMC5883P_REG_DATA_START 0x01  /* X_L,X_H,Y_L,Y_H,Z_L,Z_H - 6 byte */
#define QMC5883P_REG_STATUS     0x09  /* bit0 = DRDY */
#define QMC5883P_REG_CTRL1      0x0A  /* mode/ODR/oversample/downsample */
#define QMC5883P_REG_CTRL2      0x0B  /* soft reset (0x80) / range secimi */
#define QMC5883P_REG_SIGN       0x29  /* eksen isareti - 0x06 yazilmasi gerekiyor */

#define QMC5883P_CTRL2_SOFT_RESET   0x80
#define QMC5883P_CTRL2_RANGE_2G     0x0C  /* en hassas aralik, 15000 LSB/Gauss */

/* CTRL1: downsample(7:6)=8x, oversample(5:4)=8x, ODR(3:2)=200Hz, mode(1:0)=normal */
#define QMC5883P_CTRL1_CONFIG       0xCD

#define QMC5883P_LSB_PER_GAUSS      15000.0f
#define QMC5883P_GAUSS_TO_UT        100.0f  /* 1 Gauss = 100 uT */

typedef struct {
    float mag_x, mag_y, mag_z;  /* mikro Tesla */
} QMC5883P_Data_t;

HAL_StatusTypeDef QMC5883P_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef QMC5883P_ReadData(I2C_HandleTypeDef *hi2c, QMC5883P_Data_t *data);

/* Hard-iron kalibrasyonu: QMC5883P_ReadData'nin cikardigi sabit ofset (uT).
 * Ofsetler kartin tum yonlerde dondurulmesiyle toplanan min/max degerlerinden
 * ((min+max)/2) hesaplanmali - bu hesaplama vTaskDelay gerektirdigi icin
 * (surucu FreeRTOS'a bagimli olmamali) freertos.c/Mag_Task icinde yapiliyor,
 * burada sadece sonucun uygulanmasi/okunmasi var. Soft-iron (olcek/aci
 * bozulmasi) duzeltmesi yok, sadece hard-iron ofset. */
void QMC5883P_SetHardIronOffset(float offset_x, float offset_y, float offset_z);
void QMC5883P_GetHardIronOffset(float *offset_x, float *offset_y, float *offset_z);

#ifdef __cplusplus
}
#endif

#endif /* __QMC5883P_H__ */
