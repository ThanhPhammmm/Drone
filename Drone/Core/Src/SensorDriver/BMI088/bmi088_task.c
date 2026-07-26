#include "bmi088_task.h"
#include "bmi088.h"
#include "string.h"
#include "stdio.h"
#include "semphr.h"
#include "imu_topic.h"
#include "Const.h"
#include "lpf.h"

extern BMI088_Handle_t bmi088;
extern UART_HandleTypeDef huart1;
extern volatile uint32_t gyro_dt;

SemaphoreHandle_t imuDmaSem;
#define GYRO_LPF_CUTOFF_HZ				90.0f   /* 80-100Hz */
#define ACCEL_LPF_CUTOFF_HZ				30.0f	/* 20-50Hz	*/

#ifdef DEBUG
#define LOG_SIZE 500
static BMI088_Data_t logBuf[LOG_SIZE];
static uint32_t logIdx = 0;
#endif

#ifdef DEBUG
void BMI088_PrintData(const uint32_t *gyro_count, const uint32_t *accel_count, const volatile uint32_t *imu_dt, const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 500)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"gyro_count = %lu, accel_count = %lu, dt = %lu - ACC: %.3f %.3f %.3f | GYR: %.3f %.3f %.3f\r\n",
		*gyro_count, *accel_count, *imu_dt,
		imu->accel.x, imu->accel.y, imu->accel.z,
		imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void BMI088_PrintDataCSV(const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%lu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\r\n",
		imu->timestamp_us,
		imu->accel.x, imu->accel.y, imu->accel.z,
		imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

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
#ifdef DEBUG
    if(huart1.gState == HAL_UART_STATE_READY){
        const char *msg = (calibStatus == BMI088_OK)
            ? "IMU calib OK\r\n"
            : "IMU calib FAILED (board moved or SPI error)\r\n";
        HAL_UART_Transmit_DMA(&huart1, (uint8_t *)msg, strlen(msg));
    }
    vTaskDelay(pdMS_TO_TICKS(5000));
#endif
    static uint32_t lastCycle = 0;
    static uint32_t imuDt = 0;
    static uint8_t  dtValid = 0;
    static LPF_t gyroLPF[3];
    static LPF_t accelLPF[3];
	static uint8_t lpfInit = 0;
    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if(!lpfInit){
            LPF_Init(&gyroLPF[0], GYRO_LPF_CUTOFF_HZ);
            LPF_Init(&gyroLPF[1], GYRO_LPF_CUTOFF_HZ);
            LPF_Init(&gyroLPF[2], GYRO_LPF_CUTOFF_HZ);

            LPF_Init(&accelLPF[0], ACCEL_LPF_CUTOFF_HZ);
            LPF_Init(&accelLPF[1], ACCEL_LPF_CUTOFF_HZ);
            LPF_Init(&accelLPF[2], ACCEL_LPF_CUTOFF_HZ);

            lpfInit = 1;
        }

        bmi088.data.gyro_updated = false;
        bmi088.data.accel_updated = false;

        if(bmi088.gyroReady){
        	bmi088.gyroReady = false;
            if(BMI088_ReadGyro() != BMI088_OK) continue;
            BMI088_ParseGyro();
            bmi088.data.gyro_updated = true;

            uint32_t now = DWT->CYCCNT;
            imuDt = now - lastCycle;
            lastCycle = now;

            if(!dtValid){ // the first sample after calibration
            	dtValid = 1;
            	bmi088.data.dt = 0.0f;
            }
            else{
            	bmi088.data.dt = (float)imuDt / (float)SystemCoreClock;
            }
            bmi088.data.timestamp_us = DWT->CYCCNT / (SystemCoreClock / 1000000U);
            bmi088.data.timestamp = xTaskGetTickCount();
        }
        if(bmi088.accelReady){
        	bmi088.accelReady = false;
            if(BMI088_ReadAccel() != BMI088_OK) continue;
            BMI088_ParseAccel();
            bmi088.data.accel_updated = true;
        }

        BMI088_Convert();

        if(bmi088.data.gyro_updated){
            bmi088.data.gyro.x = LPF_Update(&gyroLPF[0], bmi088.data.gyro.x, bmi088.data.dt);
            bmi088.data.gyro.y = LPF_Update(&gyroLPF[1], bmi088.data.gyro.y, bmi088.data.dt);
            bmi088.data.gyro.z = LPF_Update(&gyroLPF[2], bmi088.data.gyro.z, bmi088.data.dt);
        }
        if(bmi088.data.accel_updated){
            bmi088.data.accel.x = LPF_Update(&accelLPF[0], bmi088.data.accel.x, bmi088.data.dt);
            bmi088.data.accel.y = LPF_Update(&accelLPF[1], bmi088.data.accel.y, bmi088.data.dt);
            bmi088.data.accel.z = LPF_Update(&accelLPF[2], bmi088.data.accel.z, bmi088.data.dt);
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
