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
#include "queue.h"
#include "semphr.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* Mutex handle */
SemaphoreHandle_t xCANMutex;

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

/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void IMU_Task(void *argument);
void Baro_Task(void *argument);
void Mag_Task(void *argument);
void CAN_TX_Task(void *argument);
void WDG_Task(void *argument);

/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void MX_FREERTOS_Init(void) {

    /* Queue oluştur */
    xIMUQueue  = xQueueCreate(10, sizeof(IMU_Data_t));
    xBaroQueue = xQueueCreate(5,  sizeof(Baro_Data_t));
    xMagQueue  = xQueueCreate(5,  sizeof(Mag_Data_t));

    /* Mutex oluştur */
    xCANMutex = xSemaphoreCreateMutex();

    /* Task'ları oluştur */
    xTaskCreate(IMU_Task,    "IMU_TASK",    256, NULL, 4, NULL);
    xTaskCreate(Baro_Task,   "BARO_TASK",   128, NULL, 3, NULL);
    xTaskCreate(Mag_Task,    "MAG_TASK",    128, NULL, 3, NULL);
    xTaskCreate(CAN_TX_Task, "CAN_TX_TASK", 256, NULL, 3, NULL);
    xTaskCreate(WDG_Task,    "WDG_TASK",    128, NULL, 1, NULL);

    /* Scheduler başlat */
    vTaskStartScheduler();
}

/* IMU Task — MPU6050 okuma, 50ms */
void IMU_Task(void *argument) {
    IMU_Data_t imu_data;
    for(;;) {
        /* TODO: MPU6050 I2C okuma buraya gelecek */
        imu_data.accel_x = 0.0f;
        imu_data.accel_y = 0.0f;
        imu_data.accel_z = 9.81f;
        imu_data.gyro_x  = 0.0f;
        imu_data.gyro_y  = 0.0f;
        imu_data.gyro_z  = 0.0f;

        xQueueSend(xIMUQueue, &imu_data, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* Baro Task — BMP180 okuma, 500ms */
void Baro_Task(void *argument) {
    Baro_Data_t baro_data;
    for(;;) {
        /* TODO: BMP180 I2C okuma buraya gelecek */
        baro_data.pressure    = 1013.25f;
        baro_data.temperature = 25.0f;
        baro_data.altitude    = 0.0f;

        xQueueSend(xBaroQueue, &baro_data, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

/* Mag Task — HMC5883L okuma, 100ms */
void Mag_Task(void *argument) {
    Mag_Data_t mag_data;
    for(;;) {
        /* TODO: HMC5883L I2C okuma buraya gelecek */
        mag_data.mag_x = 0.0f;
        mag_data.mag_y = 0.0f;
        mag_data.mag_z = 0.0f;

        xQueueSend(xMagQueue, &mag_data, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/* CAN TX Task — queue'dan al, CAN'a gönder */
void CAN_TX_Task(void *argument) {
    IMU_Data_t  imu_data;
    Baro_Data_t baro_data;
    Mag_Data_t  mag_data;

    for(;;) {
        /* IMU verisi geldi mi? */
        if(xQueueReceive(xIMUQueue, &imu_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                /* TODO: CAN frame pack ve gönder buraya gelecek */
                /* CAN ID: 0x101 — IMU data */
                xSemaphoreGive(xCANMutex);
            }
        }

        /* Baro verisi geldi mi? */
        if(xQueueReceive(xBaroQueue, &baro_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                /* TODO: CAN frame pack ve gönder buraya gelecek */
                /* CAN ID: 0x102 — Baro data */
                xSemaphoreGive(xCANMutex);
            }
        }

        /* Mag verisi geldi mi? */
        if(xQueueReceive(xMagQueue, &mag_data, pdMS_TO_TICKS(10)) == pdTRUE) {
            if(xSemaphoreTake(xCANMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                /* TODO: CAN frame pack ve gönder buraya gelecek */
                /* CAN ID: 0x103 — Mag data */
                xSemaphoreGive(xCANMutex);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* Watchdog Task — sistem sağlığı izleme, 1000ms */
void WDG_Task(void *argument) {
    for(;;) {
        /* TODO: heap monitör, IWDG refresh buraya gelecek */
        uint32_t heap_free = xPortGetFreeHeapSize();
        (void)heap_free;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

/* USER CODE END Application */

