#include "magbaro_task.h"

#define MAGBARO_WAIT_MS          50

#define MAGBARO_POWERUP_MS       50      /* both sensors finish their power-on reset */
#define MAGBARO_INIT_TRIES       5
#define MAGBARO_INIT_RETRY_MS    50

#define MAGBARO_STALL_MS         50

#define MAGBARO_SCL_PORT         GPIOB
#define MAGBARO_SCL_PIN          GPIO_PIN_10
#define MAGBARO_SDA_PORT         GPIOB
#define MAGBARO_SDA_PIN          GPIO_PIN_9
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

/* ---- calibration asked from the remote (TFT page 6) ----
 * The old values are kept here and put back on cancel or failure. */
static uint8_t            baroSvc;          /* the running baro calibration was asked from the remote */
static BMP388_GroundRef_t baroOld;
static Calib_Status_t     baroOldStatus;
static uint8_t            magSvc;
static QMC5883_Calib_t    magOld;
static Calib_Status_t     magOldStatus;

static int16_t MagBaro_S16(float v){
    if(v >  32767.0f) v =  32767.0f;
    if(v < -32767.0f) v = -32767.0f;
    return (int16_t)(v + (v >= 0.0f ? 0.5f : -0.5f));
}

/* turned x, y, z: how far each axis has swept, % of what the calibration needs */
static void Mag_Turned(int16_t r[6]){
    for(uint8_t k = 0; k < 3; k++){
        float span = qmc5883.calib.maxV[k] - qmc5883.calib.minV[k];
        float pct  = (span > 0.0f) ? span * 100.0f / QMC5883_CALIB_MIN_SPAN : 0.0f;
        r[k] = MagBaro_S16(pct > 100.0f ? 100.0f : pct);
    }
}

/* 1 = a new sample was read. woken = the INT edge said so, and its time is
 * the sample's time. */
static uint8_t Baro_Update(uint8_t woken){
    if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_WAITING && Calib_IsTurn(CALIB_SENSOR_BMP388)){
#if CALIB_BMP388_RUN
        BMP388_GroundRefReset();
        Calib_Start(CALIB_SENSOR_BMP388);
#else
        BMP388_GroundRefSet(CALIB_BMP388_GROUND_PA);
        Calib_SetStatus(CALIB_SENSOR_BMP388, CALIB_STORED);
#endif
    }

    if(CalSvc_Take(RC_CAL_BARO)){
        baroOld       = bmp388.groundRef;
        baroOldStatus = Calib_GetStatus(CALIB_SENSOR_BMP388);
        baroSvc       = 1;
        BMP388_GroundRefReset();
        Calib_Start(CALIB_SENSOR_BMP388);
    }
    if(baroSvc && CalSvc_CancelRequested(RC_CAL_BARO)){
        bmp388.groundRef = baroOld;
        Calib_SetStatus(CALIB_SENSOR_BMP388, baroOldStatus);
        baroSvc = 0;
        CalSvc_Finish(RC_CAL_BARO, TLM_CAL_CANCELLED, NULL);
    }

    if(!BMP388_DataReady() || BMP388_Read() != BMP388_OK) return 0;

    if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_RUNNING){
        BMP388_GroundRefAccumulate();
        if(baroSvc) CalSvc_Progress(RC_CAL_BARO, (uint8_t)(bmp388.groundRef.samples * 100U / BMP388_GROUND_REF_SAMPLES), NULL);
        if(bmp388.groundRef.valid){
            Calib_SetStatus(CALIB_SENSOR_BMP388, CALIB_DONE);
            if(baroSvc){
                int16_t r[6] = {0};
                r[0] = MagBaro_S16(bmp388.groundRef.pressure_pa * 0.1f);   /* Pa -> 0.1 hPa */
                baroSvc = 0;
                CalSvc_Finish(RC_CAL_BARO, TLM_CAL_OK, r);
            }
        }
    }
    else if(Calib_Usable(CALIB_SENSOR_BMP388)){
        Baro_Data_t baro;
        baro.pressure_pa   = bmp388.pressure_pa;
        baro.temperature_c = bmp388.temperature_c;
        baro.altitude_m    = bmp388.altitude_m;
        baro.timestamp_us  = woken ? Time_UsAt(bmp388.irqCycles) : Time_Us();
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

    if(CalSvc_Take(RC_CAL_COMPASS)){
        magOld       = qmc5883.calib;
        magOldStatus = Calib_GetStatus(CALIB_SENSOR_QMC5883);
        magSvc       = 1;
        QMC5883_CalibReset();
        Calib_Start(CALIB_SENSOR_QMC5883);
    }
    if(magSvc && CalSvc_CancelRequested(RC_CAL_COMPASS)){
        qmc5883.calib = magOld;
        Calib_SetStatus(CALIB_SENSOR_QMC5883, magOldStatus);
        magSvc = 0;
        CalSvc_Finish(RC_CAL_COMPASS, TLM_CAL_CANCELLED, NULL);
    }

    uint8_t got = 0;
    if(QMC5883_DataReady() && QMC5883_Read() == QMC5883_OK){
        got = 1;
        if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_RUNNING){
            QMC5883_CalibAccumulate();
            if(magSvc){
                int16_t r[6] = {0};
                Mag_Turned(r);
                CalSvc_Progress(RC_CAL_COMPASS,
                                (uint8_t)(Calib_ElapsedMs(CALIB_SENSOR_QMC5883) * 100U / CALIB_QMC5883_DURATION_MS), r);
            }
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
        uint8_t ok = (QMC5883_CalibFinish() == QMC5883_OK);
        if(magSvc){
            int16_t r[6] = {0};
            magSvc = 0;
            if(ok){
                Calib_SetStatus(CALIB_SENSOR_QMC5883, CALIB_DONE);
                r[0] = MagBaro_S16(qmc5883.calib.offset[0]);
                r[1] = MagBaro_S16(qmc5883.calib.offset[1]);
                r[2] = MagBaro_S16(qmc5883.calib.offset[2]);
                CalSvc_Finish(RC_CAL_COMPASS, TLM_CAL_OK, r);
            }
            else{
                Mag_Turned(r);                      /* which axis was short */
                qmc5883.calib = magOld;             /* asked from the remote: keep the old offsets */
                Calib_SetStatus(CALIB_SENSOR_QMC5883, magOldStatus);
                CalSvc_Finish(RC_CAL_COMPASS, TLM_CAL_TURN_MORE, r);
            }
        }
        else{
            Calib_SetStatus(CALIB_SENSOR_QMC5883, ok ? CALIB_DONE : CALIB_FAILED);
        }
    }
    return got;
}

void MagBaroTask(void *argument){
    BMP388_SetTaskHandle(xTaskGetCurrentTaskHandle());
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

    TickType_t baroSeen = xTaskGetTickCount();
    TickType_t magSeen  = baroSeen;

    while(1){
        uint8_t woken = (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(MAGBARO_WAIT_MS)) > 0U);
        TickType_t now = xTaskGetTickCount();

        if(baroOk){
            if(Baro_Update(woken) || !Calib_Finished(CALIB_SENSOR_BMP388)) baroSeen = now;
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
