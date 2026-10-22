#include "magbaro_task.h"

#define MAGBARO_TASK_RATE_HZ     100
#define MAGBARO_TASK_PERIOD_MS   (1000 / MAGBARO_TASK_RATE_HZ)

#define MAGBARO_POWERUP_MS       50      /* both sensors finish their power-on reset */
#define MAGBARO_INIT_TRIES       5
#define MAGBARO_INIT_RETRY_MS    50

#define MAGBARO_STALL_MS         250

#define MAGBARO_SCL_PORT         GPIOB
#define MAGBARO_SCL_PIN          GPIO_PIN_10
#define MAGBARO_SDA_PORT         GPIOB
#define MAGBARO_SDA_PIN          GPIO_PIN_3
#define MAGBARO_BUS_HALF_US      5       /* 100 kHz while clocking by hand */

static void MagBaro_WaitUs(uint32_t us){
    uint32_t t0 = Time_Us();
    while((uint32_t)(Time_Us() - t0) < us){}
}

static void MagBaro_BusRecover(void){
    HAL_I2C_DeInit(&hi2c2);

    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin   = MAGBARO_SCL_PIN;
    HAL_GPIO_Init(MAGBARO_SCL_PORT, &g);
    g.Pin   = MAGBARO_SDA_PIN;
    HAL_GPIO_Init(MAGBARO_SDA_PORT, &g);

    HAL_GPIO_WritePin(MAGBARO_SDA_PORT, MAGBARO_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MAGBARO_SCL_PORT, MAGBARO_SCL_PIN, GPIO_PIN_SET);
    MagBaro_WaitUs(MAGBARO_BUS_HALF_US);

    for(uint8_t i = 0; i < 9; i++){
        if(HAL_GPIO_ReadPin(MAGBARO_SDA_PORT, MAGBARO_SDA_PIN) == GPIO_PIN_SET) break;
        HAL_GPIO_WritePin(MAGBARO_SCL_PORT, MAGBARO_SCL_PIN, GPIO_PIN_RESET);
        MagBaro_WaitUs(MAGBARO_BUS_HALF_US);
        HAL_GPIO_WritePin(MAGBARO_SCL_PORT, MAGBARO_SCL_PIN, GPIO_PIN_SET);
        MagBaro_WaitUs(MAGBARO_BUS_HALF_US);
    }

    /* STOP: SDA goes high while SCL is high */
    HAL_GPIO_WritePin(MAGBARO_SCL_PORT, MAGBARO_SCL_PIN, GPIO_PIN_RESET);
    MagBaro_WaitUs(MAGBARO_BUS_HALF_US);
    HAL_GPIO_WritePin(MAGBARO_SDA_PORT, MAGBARO_SDA_PIN, GPIO_PIN_RESET);
    MagBaro_WaitUs(MAGBARO_BUS_HALF_US);
    HAL_GPIO_WritePin(MAGBARO_SCL_PORT, MAGBARO_SCL_PIN, GPIO_PIN_SET);
    MagBaro_WaitUs(MAGBARO_BUS_HALF_US);
    HAL_GPIO_WritePin(MAGBARO_SDA_PORT, MAGBARO_SDA_PIN, GPIO_PIN_SET);
    MagBaro_WaitUs(MAGBARO_BUS_HALF_US);

    HAL_I2C_Init(&hi2c2);
}

static void Baro_Restart(void){
    BMP388_GroundRef_t ref = bmp388.groundRef;
    MagBaro_BusRecover();
    BMP388_Init();
    bmp388.groundRef = ref;
}

static void Mag_Restart(void){
    QMC5883_Calib_t cal = qmc5883.calib;
    MagBaro_BusRecover();
    QMC5883_Init();
    qmc5883.calib = cal;
}

/* 1 = a new sample was read */
static uint8_t Baro_Update(void){
    if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_WAITING && Calib_IsTurn(CALIB_SENSOR_BMP388)){
#if CALIB_BMP388_RUN
        BMP388_GroundRefReset();
        Calib_Start(CALIB_SENSOR_BMP388);
#else
        BMP388_GroundRefSet(CALIB_BMP388_GROUND_PA);
        Calib_SetStatus(CALIB_SENSOR_BMP388, CALIB_STORED);
#endif
    }

    if(!BMP388_DataReady() || BMP388_Read() != BMP388_OK) return 0;

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
    return 1;
}

/* 1 = a new sample was read */
static uint8_t Mag_Update(void){
    if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_WAITING && Calib_IsTurn(CALIB_SENSOR_QMC5883)){
#if CALIB_QMC5883_RUN
        QMC5883_CalibReset();
        Calib_Start(CALIB_SENSOR_QMC5883);
#else
        QMC5883_SetOffsets(CALIB_QMC5883_OFFSET_X, CALIB_QMC5883_OFFSET_Y, CALIB_QMC5883_OFFSET_Z);
        Calib_SetStatus(CALIB_SENSOR_QMC5883, CALIB_STORED);
#endif
    }

    uint8_t got = 0;
    if(QMC5883_DataReady() && QMC5883_Read() == QMC5883_OK){
        got = 1;
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
    return got;
}

void MagBaroTask(void *argument){
    vTaskDelay(pdMS_TO_TICKS(MAGBARO_POWERUP_MS));

    uint8_t magOk  = 0;
    uint8_t baroOk = 0;
    for(uint8_t attempt = 0; attempt < MAGBARO_INIT_TRIES && !(magOk && baroOk); attempt++){
        if(attempt > 0) vTaskDelay(pdMS_TO_TICKS(MAGBARO_INIT_RETRY_MS));
        MagBaro_BusRecover();
        if(!magOk)  magOk  = (QMC5883_Init() == QMC5883_OK);
        if(!baroOk) baroOk = (BMP388_Init()  == BMP388_OK);
        if(magOk && baroOk) break;
    }

    if(!baroOk) Calib_SetStatus(CALIB_SENSOR_BMP388,  CALIB_NO_SENSOR);
    if(!magOk)  Calib_SetStatus(CALIB_SENSOR_QMC5883, CALIB_NO_SENSOR);

    if(!magOk && !baroOk){
        vTaskDelete(NULL);
    }

    TickType_t last     = xTaskGetTickCount();
    TickType_t baroSeen = last;
    TickType_t magSeen  = last;

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(MAGBARO_TASK_PERIOD_MS));
        TickType_t now = xTaskGetTickCount();

        if(baroOk){
            if(Baro_Update()) baroSeen = now;
            else if((TickType_t)(now - baroSeen) >= pdMS_TO_TICKS(MAGBARO_STALL_MS)){
                Baro_Restart();
                baroSeen = xTaskGetTickCount();
            }
        }
        if(magOk){
            if(Mag_Update()) magSeen = now;
            else if((TickType_t)(now - magSeen) >= pdMS_TO_TICKS(MAGBARO_STALL_MS)){
                Mag_Restart();
                magSeen = xTaskGetTickCount();
            }
        }
    }
}

uint8_t MagBaro_IsCalibrated(void) {
    return (qmc5883.calib.calibrated && bmp388.groundRef.valid);
}
