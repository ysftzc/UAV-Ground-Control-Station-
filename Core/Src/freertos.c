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
#include "queue.h"
#include "semphr.h"
#include <stdio.h>
#include <string.h>
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
/* USER CODE END FunctionPrototypes */

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

void MX_FREERTOS_Init(void) {

    /* Queue olustur */
    xIMUQueue  = xQueueCreate(10, sizeof(IMU_Data_t));
    xBaroQueue = xQueueCreate(5,  sizeof(Baro_Data_t));
    xMagQueue  = xQueueCreate(5,  sizeof(Mag_Data_t));

    /* Mutex olustur */
    xCANMutex  = xSemaphoreCreateMutex();
    xUARTMutex = xSemaphoreCreateMutex();

    /* Task'lari olustur */
    xTaskCreate(IMU_Task,    "IMU_TASK",    256, NULL, 4, NULL);
    xTaskCreate(Baro_Task,   "BARO_TASK",   128, NULL, 3, NULL);
    xTaskCreate(Mag_Task,    "MAG_TASK",    128, NULL, 3, NULL);
    xTaskCreate(CAN_TX_Task, "CAN_TX_TASK", 256, NULL, 3, NULL);
    xTaskCreate(WDG_Task,    "WDG_TASK",    128, NULL, 1, NULL);

    /* Scheduler baslat */
    vTaskStartScheduler();
}

/* I2C bus tarama - hangi adreste cihaz ACK veriyor gormek icin (debug) */
static void I2C_Scan(I2C_HandleTypeDef *hi2c) {
    char msg[64];
    int found = 0;

    for (uint8_t addr = 1; addr < 128; addr++) {
        if (HAL_I2C_IsDeviceReady(hi2c, (uint16_t)(addr << 1), 2, 5) == HAL_OK) {
            snprintf(msg, sizeof(msg), "[I2C] cihaz bulundu: 0x%02X\r\n", addr);
            if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
                xSemaphoreGive(xUARTMutex);
            }
            found++;
        }
    }

    if (found == 0) {
        snprintf(msg, sizeof(msg), "[I2C] bus taramasi: hicbir cihaz yanit vermedi\r\n");
        if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            xSemaphoreGive(xUARTMutex);
        }
    }
}

/* IMU Task - MPU6050 okuma, 500ms */
void IMU_Task(void *argument) {
    IMU_Data_t imu_data = {0};
    MPU6050_Data_t mpu_data;
    char msg[96];

    I2C_Scan(&hi2c1);

    /* MPU6050'yi baslat - basarisiz olursa task son okunan degerlerle devam eder */
    HAL_StatusTypeDef init_status = MPU6050_Init(&hi2c1);
    snprintf(msg, sizeof(msg), "[IMU] MPU6050 init %s\r\n",
             (init_status == HAL_OK) ? "OK" : "FAIL");
    if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
        xSemaphoreGive(xUARTMutex);
    }

    for(;;) {
        if(MPU6050_ReadData(&hi2c1, &mpu_data) == HAL_OK) {
            imu_data.accel_x = mpu_data.accel_x;
            imu_data.accel_y = mpu_data.accel_y;
            imu_data.accel_z = mpu_data.accel_z;
            imu_data.gyro_x  = mpu_data.gyro_x;
            imu_data.gyro_y  = mpu_data.gyro_y;
            imu_data.gyro_z  = mpu_data.gyro_z;
        }

        xQueueSend(xIMUQueue, &imu_data, 0);

        /* UART debug - integer olarak gonder (float printf gerektirmez) */
        snprintf(msg, sizeof(msg), "[IMU] ax:%d ay:%d az:%d gx:%d gy:%d gz:%d\r\n",
                 (int)(imu_data.accel_x * 100),
                 (int)(imu_data.accel_y * 100),
                 (int)(imu_data.accel_z * 100),
                 (int)(imu_data.gyro_x  * 100),
                 (int)(imu_data.gyro_y  * 100),
                 (int)(imu_data.gyro_z  * 100));

        if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            xSemaphoreGive(xUARTMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Baro Task - BMP180 okuma, 2000ms */
void Baro_Task(void *argument) {
    Baro_Data_t baro_data;
    char msg[64];
    for(;;) {
        /* TODO: BMP180 I2C okuma buraya gelecek */
        baro_data.pressure    = 1013.25f;
        baro_data.temperature = 25.0f;
        baro_data.altitude    = 0.0f;

        xQueueSend(xBaroQueue, &baro_data, 0);

        snprintf(msg, sizeof(msg), "[BARO] p:%d t:%d alt:%d\r\n",
                 (int)(baro_data.pressure    * 100),
                 (int)(baro_data.temperature * 100),
                 (int)(baro_data.altitude    * 100));

        if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            xSemaphoreGive(xUARTMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/* Mag Task - HMC5883L okuma, 1000ms */
void Mag_Task(void *argument) {
    Mag_Data_t mag_data;
    char msg[64];
    for(;;) {
        /* TODO: HMC5883L I2C okuma buraya gelecek */
        mag_data.mag_x = 0.0f;
        mag_data.mag_y = 0.0f;
        mag_data.mag_z = 0.0f;

        xQueueSend(xMagQueue, &mag_data, 0);

        snprintf(msg, sizeof(msg), "[MAG] mx:%d my:%d mz:%d\r\n",
                 (int)(mag_data.mag_x * 100),
                 (int)(mag_data.mag_y * 100),
                 (int)(mag_data.mag_z * 100));

        if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            xSemaphoreGive(xUARTMutex);
        }

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

/* Watchdog Task - sistem sagligi izleme + LED blink, 1000ms */
void WDG_Task(void *argument) {
    char msg[64];
    uint32_t tick = 0;
    for(;;) {
        /* PC13 LED toggle */
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);

        /* Heap monitor + UART */
        uint32_t heap_free = xPortGetFreeHeapSize();
        snprintf(msg, sizeof(msg), "[WDG] tick:%lu heap:%lu free\r\n", tick++, heap_free);

        if(xSemaphoreTake(xUARTMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 100);
            xSemaphoreGive(xUARTMutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* USER CODE END Application */
