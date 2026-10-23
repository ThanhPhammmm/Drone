#include "qmc5883_task.h"
#include "qmc5883.h"
#include "mag_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "stdio.h"

#define MAGBARO_TASK_RATE_HZ		100
#define MAGBARO_TASK_PERIOD_MS		(1000 / MAGBARO_TASK_RATE_HZ)
#define MAG_CALIB_DURATION_MS		15000

void MagTask(void *argument){
	uint8_t magOk  = (QMC5883_Init() == QMC5883_OK);
	if(!magOk){
		vTaskDelete(NULL);
	}
	if(magOk)	QMC5883_CalibReset();

    uint32_t elapsedMs = 0;
    TickType_t last = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAGBARO_TASK_PERIOD_MS));
        elapsedMs += MAGBARO_TASK_PERIOD_MS;

        if(magOk && QMC5883_DataReady()){
        	if(QMC5883_Read() == QMC5883_OK){
        		if(!qmc5883.calib.calibrated){
					QMC5883_CalibAccumulate();
					if(elapsedMs >= MAG_CALIB_DURATION_MS){
						QMC5883_CalibFinish();
					}
				}
        		else{
					Mag_Data_t mag;
					mag.x = qmc5883.field.x;
					mag.y = qmc5883.field.y;
					mag.z = qmc5883.field.z;
					mag.timestamp_us = DWT->CYCCNT / (SystemCoreClock / 1000000U);
					MagTopic_Publish(&mag);
				}
			}
		}
        /* future: if(baroOk && BMP388_Read() == BMP388_OK){ ...BaroTopic_Publish(...); } */
    }
}
