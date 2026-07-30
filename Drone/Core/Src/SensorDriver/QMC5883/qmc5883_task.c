#include "qmc5883_task.h"
#include "qmc5883.h"
#include "mag_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx.h"
#include "stdio.h"

extern UART_HandleTypeDef huart1;

#define MAGBARO_TASK_RATE_HZ		100
#define MAGBARO_TASK_PERIOD_MS		(1000 / MAGBARO_TASK_RATE_HZ)
#define MAG_CALIB_DURATION_MS		15000

void PrintCalibArgu(const QMC5883_Calib_t* calib){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f,%.6f,%.6f\r\n",
		calib->offset[0], calib->offset[1], calib->offset[2]);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

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
						PrintCalibArgu(&qmc5883.calib);
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
