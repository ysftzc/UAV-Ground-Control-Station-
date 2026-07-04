/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "can.h"
#include "usart.h"
#include "i2c.h"
#include "mpu6050.h"
#include "bmp180.h"
#include "qmc5883p.h"
#include "mavlink_bridge.h"
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <float.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* Queue handles */
QueueHandle_t xIMUQueue;
QueueHandle_t xBaroQueue;
QueueHandle_t xMagQueue;

/* Mutex handles */
SemaphoreHandle_t xCANMutex;
SemaphoreHandle_t xUARTMutex;
SemaphoreHandle_t xI2CMutex;    /* hi2c1, IMU/Baro/Mag task'lari arasinda paylasiliyor */
SemaphoreHandle_t xSensorMutex; /* g_sensor_snapshot okuma/yazma korumasi */

/* IMU/Baro/Mag task'larinin en son okudugu deger - MAVLINK_TX_TASK'in
 * kendi hizinda (queue tuketmeden, "en guncel deger" semantigiyle)
 * okuyabilmesi icin CAN yolundaki queue'lardan ayri, mutex korumali
 * paylasilan bir anlik goruntu (snapshot). */
static MAVLink_SensorSnapshot_t g_sensor_snapshot = {0};

/* Sensor data structs */
typedef struct {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
} IMU_Data_t;

typedef struct {
    float pressure;
    float temperature;
    float altitude;
} Baro_Data_t;

typedef struct {
    float mag_x, mag_y, mag_z;
} Mag_Data_t;

/* Task handle'lari - WDG_Task'tan stack high-water-mark okumak icin */
TaskHandle_t xImuTaskHandle, xBaroTaskHandle, xMagTaskHandle, xCanTaskHandle, xWdgTaskHandle, xMavlinkTaskHandle;

/* CAN handles */
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void IMU_Task(void *argument);
void Baro_Task(void *argument);
void Mag_Task(void *argument);
void CAN_TX_Task(void *argument);
void WDG_Task(void *argument);
void MAVLink_TX_Task(void *argument);
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* Debug/telemetri UART ayrimi: Release build'de (DEBUG tanimli degil) bu
 * fonksiyon no-op'a donusur, UART1 sadece MAVLink binary akisini tasir.
 * Debug build'de (DEBUG tanimli) insan-okunabilir loglar eskisi gibi basilir.
 * Gercek savunma sanayi/havacilik firmware'lerinde debug konsolu ile telemetri
 * hatti asla ayni fiziksel kanalda karistirilmaz - bu ayrim onu taklit eder. */
#ifdef DEBUG
static void Debug_UartPrint(const char *fmt, ...) {
    char msg[96];
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, sizeof(msg), fmt, args);
    va_end(args);
    if (xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        xSemaphoreGive(xUARTMutex);
    }
}
#define DBG_PRINT(...) Debug_UartPrint(__VA_ARGS__)
#else
#define DBG_PRINT(...) ((void)0)
#endif

void MX_FREERTOS_Init(void) {

    /* Queue olustur */
    xIMUQueue  = xQueueCreate(10, sizeof(IMU_Data_t));
    xBaroQueue = xQueueCreate(5,  sizeof(Baro_Data_t));
    xMagQueue  = xQueueCreate(5,  sizeof(Mag_Data_t));

    /* Mutex olustur */
    xCANMutex    = xSemaphoreCreateMutex();
    xUARTMutex   = xSemaphoreCreateMutex();
    xI2CMutex    = xSemaphoreCreateMutex();
    xSensorMutex = xSemaphoreCreateMutex();

    /* Task'lari olustur - IMU_TASK/MAG_TASK stack'leri kalibrasyon kodu
     * (ek float/struct lokal degiskenler + snprintf/HAL I2C nested cagrilar)
     * icin buyutuldu, bkz. CLAUDE.md "Bilinen Tuzak: stack overflow". 384/256
     * word'de bile tasma devam ettigi icin 512/512'ye cikarildi ve gercek
     * kullanimi olcmek icin handle'lar WDG_Task'a aktarildi. WDG_TASK'in kendi
     * stack'i de (128->256) buyutuldu - hwm print icin eklenen ikinci snprintf +
     * 5 uxTaskGetStackHighWaterMark cagrisi 128 word'de tasmaya sebep oluyordu.
     * BARO_TASK 128 word'de kalmisti (hic dokunulmamisti) - IMU/MAG/WDG buyuyunce
     * sira ona gelince (BMP180 init mesaji basilirken, tam UART transmit ortasinda
     * donma) o da tasti; 384 word'e cikarildi. Heap 10240'ta hala ~1KB marj var. */
    xTaskCreate(IMU_Task,     "IMU_TASK",     512, NULL, 4, &xImuTaskHandle);
    xTaskCreate(Baro_Task,    "BARO_TASK",    384, NULL, 3, &xBaroTaskHandle);
    xTaskCreate(Mag_Task,     "MAG_TASK",     512, NULL, 3, &xMagTaskHandle);
    xTaskCreate(CAN_TX_Task,  "CAN_TX_TASK",  256, NULL, 3, &xCanTaskHandle);
    /* mavlink_message_t + mavlink_msg_to_send_buffer'in buf[MAVLINK_MAX_PACKET_LEN]
     * dizisi (~280 byte) iki ayri fonksiyon frame'inde ust uste biniyor -
     * IMU/MAG'da yasadigimiz stack-tasma dersini burada da uygulayip 512
     * word'den basliyoruz, tahmin edip kucuk verip sonra buyutmuyoruz. */
    xTaskCreate(MAVLink_TX_Task, "MAVLINK_TASK", 512, NULL, 3, &xMavlinkTaskHandle);
    xTaskCreate(WDG_Task,     "WDG_TASK",     256, NULL, 1, &xWdgTaskHandle);

    /* Scheduler baslat */
    vTaskStartScheduler();
}

/* I2C bus tarama - hangi adreste cihaz ACK veriyor gormek icin (debug) */
static void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    int found = 0;

    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
            DBG_PRINT("[I2C] cihaz bulundu: 0x%02X\r\n", addr);
            found++;
        }
    }

    if (found == 0) {
        DBG_PRINT("[I2C] bus taramasi: hicbir cihaz yanit vermedi\r\n");
    }
}

/* IMU Task - MPU6050 okuma, 500ms */
void IMU_Task(void *argument) {
    IMU_Data_t imu_data = {0};
    MPU6050_Data_t mpu_data;

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    I2C_Scan(&hi2c1);

    /* MPU6050'yi baslat - basarisiz olursa task son okunan degerlerle devam eder */
    HAL_StatusTypeDef init_status = MPU6050_Init(&hi2c1);

    /* Gyro bias kalibrasyonu - kart bu sirada SABIT durmali (~1s, 200 ornek) */
    /* Release build'de DBG_PRINT no-op oldugu icin bu sadece debug amacli */
    HAL_StatusTypeDef gyro_cal_status __attribute__((unused)) = HAL_ERROR;
    if (init_status == HAL_OK) {
        gyro_cal_status = MPU6050_CalibrateGyro(&hi2c1, 200);
    }
    xSemaphoreGive(xI2CMutex);

    DBG_PRINT("[IMU] MPU6050 init %s\r\n", (init_status == HAL_OK) ? "OK" : "FAIL");

    if (init_status == HAL_OK) {
        float bx, by, bz;
        MPU6050_GetGyroBias(&bx, &by, &bz);
        DBG_PRINT("[IMU] gyro kalibrasyon %s bias gx:%d gy:%d gz:%d\r\n",
                  (gyro_cal_status == HAL_OK) ? "OK" : "FAIL",
                  (int)(bx * 100), (int)(by * 100), (int)(bz * 100));
    }

    for(;;) {
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        HAL_StatusTypeDef read_status = MPU6050_ReadData(&hi2c1, &mpu_data);
        xSemaphoreGive(xI2CMutex);
        if(read_status == HAL_OK) {
            imu_data.accel_x = mpu_data.accel_x;
            imu_data.accel_y = mpu_data.accel_y;
            imu_data.accel_z = mpu_data.accel_z;
            imu_data.gyro_x  = mpu_data.gyro_x;
            imu_data.gyro_y  = mpu_data.gyro_y;
            imu_data.gyro_z  = mpu_data.gyro_z;

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_sensor_snapshot.accel_x = imu_data.accel_x;
                g_sensor_snapshot.accel_y = imu_data.accel_y;
                g_sensor_snapshot.accel_z = imu_data.accel_z;
                g_sensor_snapshot.gyro_x  = imu_data.gyro_x;
                g_sensor_snapshot.gyro_y  = imu_data.gyro_y;
                g_sensor_snapshot.gyro_z  = imu_data.gyro_z;
                xSemaphoreGive(xSensorMutex);
            }
        }

        xQueueSend(xIMUQueue, &imu_data, 0);

        /* UART debug - integer olarak gonder (float printf gerektirmez) */
        DBG_PRINT("[IMU] ax:%d ay:%d az:%d gx:%d gy:%d gz:%d\r\n",
                  (int)(imu_data.accel_x * 100),
                  (int)(imu_data.accel_y * 100),
                  (int)(imu_data.accel_z * 100),
                  (int)(imu_data.gyro_x  * 100),
                  (int)(imu_data.gyro_y  * 100),
                  (int)(imu_data.gyro_z  * 100));

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Baro Task - BMP180 okuma, 2000ms */
void Baro_Task(void *argument) {
    Baro_Data_t baro_data = {0};
    BMP180_Data_t bmp_data;

    /* BMP180'i baslat - basarisiz olursa task son okunan degerlerle devam eder */
    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    /* Release build'de DBG_PRINT no-op oldugu icin bu sadece debug amacli */
    HAL_StatusTypeDef init_status __attribute__((unused)) = BMP180_Init(&hi2c1);
    xSemaphoreGive(xI2CMutex);
    DBG_PRINT("[BARO] BMP180 init %s\r\n", (init_status == HAL_OK) ? "OK" : "FAIL");

    for(;;) {
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        HAL_StatusTypeDef read_status = BMP180_ReadData(&hi2c1, &bmp_data);
        xSemaphoreGive(xI2CMutex);
        if(read_status == HAL_OK) {
            baro_data.pressure    = bmp_data.pressure;
            baro_data.temperature = bmp_data.temperature;
            baro_data.altitude    = bmp_data.altitude;

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_sensor_snapshot.pressure    = baro_data.pressure;
                g_sensor_snapshot.temperature = baro_data.temperature;
                g_sensor_snapshot.altitude    = baro_data.altitude;
                xSemaphoreGive(xSensorMutex);
            }
        }

        xQueueSend(xBaroQueue, &baro_data, 0);

        DBG_PRINT("[BARO] p:%d t:%d alt:%d\r\n",
                  (int)(baro_data.pressure    * 100),
                  (int)(baro_data.temperature * 100),
                  (int)(baro_data.altitude    * 100));

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Mag Task - QMC5883P okuma, 1000ms */
void Mag_Task(void *argument) {
    Mag_Data_t mag_data = {0};
    QMC5883P_Data_t qmc_data;

    xSemaphoreTake(xI2CMutex, portMAX_DELAY);
    HAL_StatusTypeDef init_status = QMC5883P_Init(&hi2c1);
    xSemaphoreGive(xI2CMutex);

    DBG_PRINT("[MAG] QMC5883P init %s\r\n", (init_status == HAL_OK) ? "OK" : "FAIL");

    /* Hard-iron kalibrasyonu: karti 10sn boyunca tum yonlerde dondurup
     * min/max eksen degerlerinden ofset = (min+max)/2 hesapla */
    if (init_status == HAL_OK) {
        DBG_PRINT("[MAG] kalibrasyon basliyor - karti 10sn tum yonlerde dondur\r\n");

        float min_x = FLT_MAX, max_x = -FLT_MAX;
        float min_y = FLT_MAX, max_y = -FLT_MAX;
        float min_z = FLT_MAX, max_z = -FLT_MAX;

        for (int i = 0; i < 100; i++) {
            xSemaphoreTake(xI2CMutex, portMAX_DELAY);
            HAL_StatusTypeDef cal_read = QMC5883P_ReadData(&hi2c1, &qmc_data);
            xSemaphoreGive(xI2CMutex);
            if (cal_read == HAL_OK) {
                if (qmc_data.mag_x < min_x) min_x = qmc_data.mag_x;
                if (qmc_data.mag_x > max_x) max_x = qmc_data.mag_x;
                if (qmc_data.mag_y < min_y) min_y = qmc_data.mag_y;
                if (qmc_data.mag_y > max_y) max_y = qmc_data.mag_y;
                if (qmc_data.mag_z < min_z) min_z = qmc_data.mag_z;
                if (qmc_data.mag_z > max_z) max_z = qmc_data.mag_z;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        if (max_x > min_x && max_y > min_y && max_z > min_z) {
            QMC5883P_SetHardIronOffset((min_x + max_x) / 2.0f, (min_y + max_y) / 2.0f, (min_z + max_z) / 2.0f);
        }

        float ox, oy, oz;
        QMC5883P_GetHardIronOffset(&ox, &oy, &oz);
        DBG_PRINT("[MAG] kalibrasyon tamam ofset x:%d y:%d z:%d\r\n",
                  (int)(ox * 100), (int)(oy * 100), (int)(oz * 100));
    }

    for(;;) {
        xSemaphoreTake(xI2CMutex, portMAX_DELAY);
        HAL_StatusTypeDef read_status = QMC5883P_ReadData(&hi2c1, &qmc_data);
        xSemaphoreGive(xI2CMutex);
        if(read_status == HAL_OK) {
            mag_data.mag_x = qmc_data.mag_x;
            mag_data.mag_y = qmc_data.mag_y;
            mag_data.mag_z = qmc_data.mag_z;

            if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                g_sensor_snapshot.mag_x = mag_data.mag_x;
                g_sensor_snapshot.mag_y = mag_data.mag_y;
                g_sensor_snapshot.mag_z = mag_data.mag_z;
                xSemaphoreGive(xSensorMutex);
            }
        }

        xQueueSend(xMagQueue, &mag_data, 0);

        DBG_PRINT("[MAG] mx:%d my:%d mz:%d\r\n",
                  (int)(mag_data.mag_x * 100),
                  (int)(mag_data.mag_y * 100),
                  (int)(mag_data.mag_z * 100));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* CAN TX Task - queue'dan al, CAN'a gonder */
void CAN_TX_Task(void *argument) {
    IMU_Data_t  imu_data;
    Baro_Data_t baro_data;
    Mag_Data_t  mag_data;

    /* CAN filtreyi baslat - tum mesajlari kabul et */
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

    /* CAN'i baslat */
    HAL_CAN_Start(&hcan);

    for(;;) {
        /* IMU verisi geldi mi? */
        if(xQueueReceive(xIMUQueue, &imu_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {

                TxHeader.StdId = 0x101;
                TxHeader.IDE   = CAN_ID_STD;
                TxHeader.RTR   = CAN_RTR_DATA;
                TxHeader.DLC   = 8;

                int16_t ax = (int16_t)(imu_data.accel_x * 100);
                int16_t ay = (int16_t)(imu_data.accel_y * 100);
                int16_t az = (int16_t)(imu_data.accel_z * 100);

                TxData[0] = (ax >> 8) & 0xFF;
                TxData[1] = ax & 0xFF;
                TxData[2] = (ay >> 8) & 0xFF;
                TxData[3] = ay & 0xFF;
                TxData[4] = (az >> 8) & 0xFF;
                TxData[5] = az & 0xFF;
                TxData[6] = 0x00;
                TxData[7] = 0x00;

                HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
                xSemaphoreGive(xCANMutex);
            }
        }

        /* Baro verisi geldi mi? */
        if(xQueueReceive(xBaroQueue, &baro_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                /* TODO: CAN ID: 0x102 - Baro data */
                xSemaphoreGive(xCANMutex);
            }
        }

        /* Mag verisi geldi mi? */
        if(xQueueReceive(xMagQueue, &mag_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                /* TODO: CAN ID: 0x103 - Mag data */
                xSemaphoreGive(xCANMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* MAVLink TX Task - HEARTBEAT (1Hz, zorunlu) + HIL_SENSOR (10Hz), UART uzerinden.
 * HIL_SENSOR, PX4 SITL'in HIL modunun sensor verisi icin bekledigi mesaj -
 * roadmap'teki "fiziksel IMU -> PX4 SITL" hedefine dogrudan hizmet eder.
 * NOT: IMU/Baro/Mag task'lari kendi periyotlarinda (500/2000/1000ms) tazeleniyor;
 * bu task 10Hz'de gonderim yapar ama tazelenmeler arasinda ayni degeri tekrar
 * yollar (tutma/hold semantigi, standart bir HIL bridge davranisi). PX4'un
 * gercek HIL modu >100Hz IMU bekler - bu, sensor donanimimizin fiziksel
 * limiti; ileride IMU_Task periyodu kisaltilarak iyilestirilebilir. */
void MAVLink_TX_Task(void *argument) {
    MAVLink_SensorSnapshot_t snapshot = {0};
    uint32_t iteration = 0;

    for(;;) {
        if (xSemaphoreTake(xSensorMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            snapshot = g_sensor_snapshot;
            xSemaphoreGive(xSensorMutex);
        }

        MAVLink_SendHilSensor(&huart1, xUARTMutex, &snapshot, (uint64_t)HAL_GetTick() * 1000ULL);

        /* HEARTBEAT 1Hz (10 iterasyonda bir, 10*100ms=1000ms) */
        if (iteration % 10 == 0) {
            MAVLink_SendHeartbeat(&huart1, xUARTMutex);
        }
        iteration++;

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* Watchdog Task - sistem sagligi izleme + LED blink, 1000ms */
void WDG_Task(void *argument) {
    /* Release build'de DBG_PRINT no-op oldugu icin bu sadece debug amacli */
    uint32_t tick __attribute__((unused)) = 0;
    for(;;) {
        /* PC13 LED toggle */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        /* Heap monitor + UART */
        uint32_t heap_free __attribute__((unused)) = xPortGetFreeHeapSize();
        DBG_PRINT("[WDG] tick:%lu heap:%lu free\r\n", tick, heap_free);
        tick++;

        /* Stack high-water-mark (kalan minimum bos stack, WORD cinsinden) -
         * ilk birkac saniyede stabillesir (kalibrasyon fazi bitince), tuning
         * icin gercek rakamlari gormek icin gecici olarak eklendi */
        DBG_PRINT("[WDG] hwm(word) imu:%u baro:%u mag:%u can:%u mav:%u wdg:%u\r\n",
                  (unsigned)uxTaskGetStackHighWaterMark(xImuTaskHandle),
                  (unsigned)uxTaskGetStackHighWaterMark(xBaroTaskHandle),
                  (unsigned)uxTaskGetStackHighWaterMark(xMagTaskHandle),
                  (unsigned)uxTaskGetStackHighWaterMark(xCanTaskHandle),
                  (unsigned)uxTaskGetStackHighWaterMark(xMavlinkTaskHandle),
                  (unsigned)uxTaskGetStackHighWaterMark(xWdgTaskHandle));

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* Stack bir task'in sinirini astiginda buraya duser (configCHECK_FOR_STACK_OVERFLOW=2) -
 * interrupt'lari kapatip LED'i hizli yanip sondurerek "coktu" durumunu normal calismadan
 * (1sn'de bir toggle) ve tam donmadan (surekli yanik) ayirt edilebilir kilar. */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    taskDISABLE_INTERRUPTS();
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 200000; i++) { }
    }
}

/* Heap dolduğunda (pvPortMalloc basarisiz) buraya duser - stack overflow hook'undan
 * daha hizli bir blink deseniyle ayirt edilir. */
void vApplicationMallocFailedHook(void) {
    taskDISABLE_INTERRUPTS();
    for (;;) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        for (volatile uint32_t i = 0; i < 50000; i++) { }
    }
}

/* USER CODE END Application */
