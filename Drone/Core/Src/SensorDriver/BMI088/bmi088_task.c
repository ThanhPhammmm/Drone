#include "bmi088_task.h"

extern BMI088_Handle_t bmi088;
extern volatile uint32_t gyro_dt;
extern UART_HandleTypeDef huart1;

SemaphoreHandle_t imuDmaSem;
#define GYRO_LPF_CUTOFF_HZ				90.0f   /* 80-100Hz */
#define GYRO_SAMPLE_RATE_HZ				2000.0f /* BMI088_GYRO_ODR_2000_* in bmi088.c */
#define ACCEL_LPF_CUTOFF_HZ				30.0f	/* 20-50Hz	*/

#ifdef DEBUG
#define LOG_SIZE 500
//static BMI088_Data_t logBuf[LOG_SIZE];
//static uint32_t logIdx = 0;
#endif

static BaseType_t  IMU_Signal_Init(void) {
    imuDmaSem = xSemaphoreCreateBinary();
    return (imuDmaSem != NULL) ? pdPASS : pdFAIL;
}

void IMUTask(void *argument){
    if(IMU_Signal_Init() != pdPASS) vTaskDelete(NULL);

    BMI088_SetTaskHandle(xTaskGetCurrentTaskHandle());
    if(BMI088_Init() != BMI088_OK) vTaskDelete(NULL);

    BMI088_Status_t calibStatus = BMI088_Calibrate(BMI088_CALIB_DEFAULT_SAMPLES);
    (void)calibStatus;
//#ifdef DEBUG
//    if(huart1.gState == HAL_UART_STATE_READY){
//        const char *msg = (calibStatus == BMI088_OK)
//            ? "IMU calib OK\r\n"
//            : "IMU calib FAILED (board moved or SPI error)\r\n";
//        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)msg, strlen(msg));
//    }
//    vTaskDelay(pdMS_TO_TICKS(5000));
//#endif
    static uint32_t lastGyroCycles  = 0;
    static uint32_t lastAccelCycles = 0;
    static uint8_t  gyroDtValid     = 0;
    static uint8_t  accelDtValid    = 0;
    static float    accelDt         = 0.0f;
    static Biquad_t gyroLPF[3];
    static LPF_t accelLPF[3];
	static uint8_t lpfInit = 0;
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(!lpfInit){
            Biquad_InitLowpass(&gyroLPF[0], GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE_HZ);
            Biquad_InitLowpass(&gyroLPF[1], GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE_HZ);
            Biquad_InitLowpass(&gyroLPF[2], GYRO_LPF_CUTOFF_HZ, GYRO_SAMPLE_RATE_HZ);

            LPF_Init(&accelLPF[0], ACCEL_LPF_CUTOFF_HZ);
            LPF_Init(&accelLPF[1], ACCEL_LPF_CUTOFF_HZ);
            LPF_Init(&accelLPF[2], ACCEL_LPF_CUTOFF_HZ);

            lpfInit = 1;
        }

        bmi088.data.gyro_updated = false;
        bmi088.data.accel_updated = false;

        if(bmi088.gyroReady){
        	bmi088.gyroReady = false;
        	uint32_t sampleCycles = bmi088.gyroIrqCycles;
            if(BMI088_ReadGyro() != BMI088_OK) continue;
            BMI088_ParseGyro();
            bmi088.data.gyro_updated = true;

            bmi088.data.dt = gyroDtValid ? (float)(sampleCycles - lastGyroCycles) / (float)SystemCoreClock : 0.0f;
            gyroDtValid    = 1;
            lastGyroCycles = sampleCycles;

            bmi088.data.timestamp_us = Time_UsAt(sampleCycles);
            bmi088.data.timestamp = xTaskGetTickCount();
        }
        if(bmi088.accelReady){
        	bmi088.accelReady = false;
        	uint32_t sampleCycles = bmi088.accelIrqCycles;
            if(BMI088_ReadAccel() != BMI088_OK) continue;
            BMI088_ParseAccel();
            bmi088.data.accel_updated = true;

            accelDt = accelDtValid ? (float)(sampleCycles - lastAccelCycles) / (float)SystemCoreClock : 0.0f;
            accelDtValid    = 1;
            lastAccelCycles = sampleCycles;
        }

        //BMI088_Convert(); Comment for now

        if(bmi088.data.gyro_updated){
        	BMI088_Gyro_Convert();
        }
        if(bmi088.data.accel_updated){
        	BMI088_Accel_Convert();
        }

        if(bmi088.data.gyro_updated){
            bmi088.data.gyro.x = Biquad_Update(&gyroLPF[0], bmi088.data.gyro.x);
            bmi088.data.gyro.y = Biquad_Update(&gyroLPF[1], bmi088.data.gyro.y);
            bmi088.data.gyro.z = Biquad_Update(&gyroLPF[2], bmi088.data.gyro.z);
        }
        if(bmi088.data.accel_updated){
            bmi088.data.accel.x = LPF_Update(&accelLPF[0], bmi088.data.accel.x, accelDt);
            bmi088.data.accel.y = LPF_Update(&accelLPF[1], bmi088.data.accel.y, accelDt);
            bmi088.data.accel.z = LPF_Update(&accelLPF[2], bmi088.data.accel.z, accelDt);
        }
#ifdef DEBUG
//        if(bmi088.data.gyro_updated && logIdx < LOG_SIZE){
//            logBuf[logIdx++] = bmi088.data;
//        }
//
//        if(logIdx == LOG_SIZE){
//            for(int i = 0; i < LOG_SIZE; i++){
//                BMI088_PrintDataCSV(&logBuf[i]);
//                vTaskDelay(pdMS_TO_TICKS(10));
//            }
//            logIdx++;
//        }
        //BMI088_PrintData(&gyro_count, &accel_count, &imu_dt, &bmi088.data);
//        BMI088_PrintDataCSV(&bmi088.data);
#endif
        //publish to topic
        if(bmi088.data.gyro_updated){
            IMUTopic_Publish(&bmi088.data);
        }
    }
}
