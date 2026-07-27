#include "qmc5883_task.h"
#include "qmc5883.h"
#include "mag_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"

#define MAG_TASK_PERIOD_MS      20
#define MAG_CALIB_DURATION_MS   15000

void MagTask(void *argument){
    if(QMC5883_Init() != QMC5883_OK){
        vTaskDelete(NULL);
    }
    QMC5883_Calibrate(MAG_CALIB_DURATION_MS);

    TickType_t last = xTaskGetTickCount();
    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAG_TASK_PERIOD_MS));

        if(QMC5883_Read() == QMC5883_OK){
            Mag_Data_t mag;
            mag.x = qmc5883.field.x;
            mag.y = qmc5883.field.y;
            mag.z = qmc5883.field.z;
            mag.timestamp_us = DWT->CYCCNT / (SystemCoreClock / 1000000U);
            MagTopic_Publish(&mag);
        }
        /* future: if(baroOk && BMP388_Read() == BMP388_OK){ ...BaroTopic_Publish(...); } */
    }
}
