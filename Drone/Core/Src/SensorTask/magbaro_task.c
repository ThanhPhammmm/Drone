#include "magbaro_task.h"

#define MAGBARO_TASK_RATE_HZ     100
#define MAGBARO_TASK_PERIOD_MS   (1000 / MAGBARO_TASK_RATE_HZ)

static void Baro_Update(void){
    if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_WAITING && Calib_IsTurn(CALIB_SENSOR_BMP388)){
#if CALIB_BMP388_RUN
        BMP388_GroundRefReset();
        Calib_Start(CALIB_SENSOR_BMP388);
#else
        BMP388_GroundRefSet(CALIB_BMP388_GROUND_PA);
        Calib_SetStatus(CALIB_SENSOR_BMP388, CALIB_STORED);
#endif
    }

    if(!BMP388_DataReady() || BMP388_Read() != BMP388_OK) return;

    if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_RUNNING){
        BMP388_GroundRefAccumulate();
        if(bmp388.groundRef.valid) Calib_SetStatus(CALIB_SENSOR_BMP388, CALIB_DONE);
    }
    else if(Calib_Usable(CALIB_SENSOR_BMP388)){
        Baro_Data_t baro;
        baro.pressure_pa   = bmp388.pressure_pa;
        baro.temperature_c = bmp388.temperature_c;
        baro.altitude_m    = bmp388.altitude_m;
        baro.timestamp_us  = Time_Us();
        BaroTopic_Publish(&baro);
    }
}

static void Mag_Update(void){
    if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_WAITING && Calib_IsTurn(CALIB_SENSOR_QMC5883)){
#if CALIB_QMC5883_RUN
        QMC5883_CalibReset();
        Calib_Start(CALIB_SENSOR_QMC5883);
#else
        QMC5883_SetOffsets(CALIB_QMC5883_OFFSET_X, CALIB_QMC5883_OFFSET_Y, CALIB_QMC5883_OFFSET_Z);
        Calib_SetStatus(CALIB_SENSOR_QMC5883, CALIB_STORED);
#endif
    }

    if(QMC5883_DataReady() && QMC5883_Read() == QMC5883_OK){
        if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_RUNNING){
            QMC5883_CalibAccumulate();
        }
        else if(Calib_Usable(CALIB_SENSOR_QMC5883)){
            Mag_Data_t mag;
            mag.x = qmc5883.field.x;
            mag.y = qmc5883.field.y;
            mag.z = qmc5883.field.z;
            mag.timestamp_us = Time_Us();
            MagTopic_Publish(&mag);
        }
    }

    if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_RUNNING &&
       Calib_ElapsedMs(CALIB_SENSOR_QMC5883) >= CALIB_QMC5883_DURATION_MS){
        /* an axis that did not sweep far enough = the drone was not turned:
         * those offsets would be noise, so the compass stays off */
        Calib_SetStatus(CALIB_SENSOR_QMC5883,
                        (QMC5883_CalibFinish() == QMC5883_OK) ? CALIB_DONE : CALIB_FAILED);
    }
}

void MagBaroTask(void *argument){
    uint8_t magOk  = (QMC5883_Init() == QMC5883_OK);
    uint8_t baroOk = (BMP388_Init()  == BMP388_OK);

    if(!baroOk) Calib_SetStatus(CALIB_SENSOR_BMP388,  CALIB_NO_SENSOR);
    if(!magOk)  Calib_SetStatus(CALIB_SENSOR_QMC5883, CALIB_NO_SENSOR);

    if(!magOk && !baroOk){
        vTaskDelete(NULL);
    }

    TickType_t last = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAGBARO_TASK_PERIOD_MS));

        if(baroOk) Baro_Update();
        if(magOk)  Mag_Update();
    }
}

uint8_t MagBaro_IsCalibrated(void) {
    return (qmc5883.calib.calibrated && bmp388.groundRef.valid);
}
