#include "magbaro_task.h"
#include "qmc5883.h"
#include "bmp388.h"
#include "mag_topic.h"
#include "baro_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stm32f4xx.h>

#define MAGBARO_TASK_RATE_HZ     100
#define MAGBARO_TASK_PERIOD_MS   (1000 / MAGBARO_TASK_RATE_HZ)
#define MAG_CALIB_DURATION_MS    15000

static uint32_t MagBaro_TimestampUs(void){
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

void MagBaroTask(void *argument){
    uint8_t magOk  = (QMC5883_Init() == QMC5883_OK);
    uint8_t baroOk = (BMP388_Init()  == BMP388_OK);

    if(!magOk && !baroOk){
        vTaskDelete(NULL);
    }

    if(magOk)  QMC5883_CalibReset();
    if(baroOk) BMP388_GroundRefReset();

    uint32_t elapsedMs = 0;
    TickType_t last = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAGBARO_TASK_PERIOD_MS));
        elapsedMs += MAGBARO_TASK_PERIOD_MS;

        /* ---------------- barometer ---------------- */
        if(baroOk && BMP388_DataReady()){
            if(BMP388_Read() == BMP388_OK){
                if(!bmp388.groundRef.valid){
                    BMP388_GroundRefAccumulate();
                }
                else{
                    Baro_Data_t baro;
                    baro.pressure_pa   = bmp388.pressure_pa;
                    baro.temperature_c = bmp388.temperature_c;
                    baro.altitude_m    = bmp388.altitude_m;
                    baro.timestamp_us  = MagBaro_TimestampUs();
                    BaroTopic_Publish(&baro);
                }
            }
        }

        /* ---------------- magnetometer ---------------- */
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
                    mag.timestamp_us = MagBaro_TimestampUs();
                    MagTopic_Publish(&mag);
                }
            }
        }
    }
}
