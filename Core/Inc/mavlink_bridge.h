/* MAVLink v2 koprusu - resmi mavlink/c_library_v2 (common dialect) ile
 * HEARTBEAT ve HIL_SENSOR mesajlarini UART uzerinden gonderir.
 *
 * HIL_SENSOR ozellikle secildi: PX4 SITL'in HIL modunun gercek/simule
 * sensor verisini almak icin bekledigi mesaj budur (roadmap'teki
 * "fiziksel IMU -> PX4 SITL" hedefiyle dogrudan uyumlu, generic
 * ATTITUDE/RAW_IMU gibi sadece izlenebilirlik amacli mesajlar degil). */
#ifndef MAVLINK_BRIDGE_H
#define MAVLINK_BRIDGE_H

#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

/* Surucu ciktilariyla ayni birimler (g, deg/s, uT, hPa, C, m) -
 * SI donusumu mavlink_bridge.c icinde yapilir, cagiran taraf donusum
 * yapmaz. */
typedef struct {
    float accel_x, accel_y, accel_z;   /* g */
    float gyro_x, gyro_y, gyro_z;      /* deg/s */
    float mag_x, mag_y, mag_z;         /* uT */
    float pressure;                    /* hPa */
    float temperature;                 /* C */
    float altitude;                    /* m */
} MAVLink_SensorSnapshot_t;

/* WDG_Task'in uxTaskGetStackHighWaterMark() ile hesapladigi gercek stack
 * marjlari (WORD cinsinden) + heap. Onceden sadece Debug UART metnine
 * (DBG_PRINT, Release'de no-op) giden bu veriyi PC'ye tasimak icin. */
typedef struct {
    uint32_t imu_hwm, baro_hwm, mag_hwm, can_hwm, mav_hwm, wdg_hwm; /* word */
    uint32_t heap_free;                                             /* byte */
} MAVLink_TaskHealth_t;

void MAVLink_SendHeartbeat(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex);

void MAVLink_SendHilSensor(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                            const MAVLink_SensorSnapshot_t *snapshot, uint64_t time_usec);

/* NAMED_VALUE_INT dizisi olarak gonderir (custom mesaj tanimlamaya/mavgen
 * calistirmaya gerek kalmadan, zaten vendored olan common dialect'ten) -
 * her task icin bir mesaj, name alaninda task kisaltmasi (ör. "imu",
 * "heap"), value alaninda gercek deger. */
void MAVLink_SendTaskHealth(UART_HandleTypeDef *huart, SemaphoreHandle_t uart_mutex,
                             const MAVLink_TaskHealth_t *health, uint32_t time_boot_ms);

#endif /* MAVLINK_BRIDGE_H */
