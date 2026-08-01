#include <attitude_topic.h>
#include "attitude_estimator_task.h"
#include "mahony.h"
#include "imu_topic.h"
#include "mag_topic.h"
#include "Const.h"
#include <stdio.h>
#include "bmi088.h"

#define MAG_MAX_AGE_US   20000U

static Mahony_t mahony;
AttitudeEstimator_Handle_t attitudeEstimator;

extern UART_HandleTypeDef huart1;
void BMI088_PrintAttitude(const AttitudeEstimator_Handle_t* attitudeEstimator){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 20)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f,%.6f,%.6f\r\n",
		attitudeEstimator->data.roll,attitudeEstimator->data.pitch, attitudeEstimator->data.yaw);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	attitudeEstimator.attitudeTask = handle;
}

void AttitudeEstimatorTask(void *argument){
	AttitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	IMUTopic_Subscribe(attitudeEstimator.attitudeTask, ATTITUDE_ESTIMATOR_ID_TASK);

    BMI088_Data_t imu;
    Mahony_Init(&mahony,1.0f,0.05f);

    while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(IMUTopic_Copy(&imu) != pdPASS) continue;

        float ax = imu.accel.x;
        float ay = imu.accel.y;
        float az = imu.accel.z;

		Mag_Data_t mag = {0};
		uint8_t magValid = 0;
		if(MagTopic_Copy(&mag, 0) == pdPASS && mag.timestamp_us != 0){
		    uint32_t age_us = imu.timestamp_us - mag.timestamp_us;
		    if(age_us < MAG_MAX_AGE_US){
		        magValid = 1;
		    }
		}
		float magVec[3] = { mag.x, mag.y, mag.z };

		Mahony_Update(&mahony, &imu, magVec, magValid);

		Attitude_Data_t *attitude = &attitudeEstimator.data;

		Mahony_GetQuaternion(&mahony, attitude->q);
		attitude->q0 = attitude->q[0];
		attitude->q1 = attitude->q[1];
		attitude->q2 = attitude->q[2];
		attitude->q3 = attitude->q[3];

		Mahony_GetEuler(&mahony, &attitude->roll, &attitude->pitch, &attitude->yaw);

	    float rate[3];
	    Mahony_GetRate(&mahony, rate);

		attitude->rollRate  = rate[0];
		attitude->pitchRate = rate[1];
		attitude->yawRate   = rate[2];

		attitude->gyroBiasX = mahony.bias[0];
		attitude->gyroBiasY = mahony.bias[1];
		attitude->gyroBiasZ = mahony.bias[2];

//        float q0 = attitude->q0, q1 = attitude->q1;
//        float q2 = attitude->q2, q3 = attitude->q3;
//
//        float rx = 2.0f * (q1*q3 - q0*q2);
//        float ry = 2.0f * (q0*q1 + q2*q3);
//        float rz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
//
//        float fDown = rx*ax + ry*ay + rz*az;
//
//        attitude->accelUp = -(fDown + BMI088_G);

        attitude->dt = imu.dt;
		attitude->timestamp_us = imu.timestamp_us;

		AttitudeTopic_Publish(attitude);
		BMI088_PrintAttitude(&attitudeEstimator);
    }
}
