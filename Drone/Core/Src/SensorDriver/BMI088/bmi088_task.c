#include "bmi088_task.h"
#include "bmi088.h"
#include "string.h"
#include "stdio.h"
#include "semphr.h"

extern BMI088_Handle_t bmi088;
extern UART_HandleTypeDef huart1;

SemaphoreHandle_t imuDmaSem;

#ifdef DEBUG
void BMI088_PrintData(const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"ACC: %.3f %.3f %.3f | GYR: %.3f %.3f %.3f\r\n",
		imu->accel.x, imu->accel.y, imu->accel.z,
		imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK)
	{
		last_print_time = current_time;
	}
}
#endif


static void IMU_Signal_Init(void) {
    imuDmaSem = xSemaphoreCreateBinary();
}

void IMUTask(void *argument){
    BMI088_SetTaskHandle(xTaskGetCurrentTaskHandle());
    IMU_Signal_Init();
    BMI088_Init();

    while(1){
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(bmi088.gyroReady){
        	bmi088.gyroReady = false;

            if(BMI088_ReadGyro() != BMI088_OK) continue;
            BMI088_ParseGyro();
        }
        if(bmi088.accelReady){
        	bmi088.accelReady = false;
            if(BMI088_ReadAccel() != BMI088_OK) continue;
            BMI088_ParseAccel();
        }
        bmi088.data.timestamp = xTaskGetTickCount();
        BMI088_Convert();
#ifdef DEBUG
        BMI088_PrintData(&bmi088.data);
#endif
        //publish to topic
    }
}
