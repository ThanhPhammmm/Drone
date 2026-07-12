#include <attitude_topic.h>
#include "attitude_estimator_task.h"
#include "mahony.h"
#include "imu_topic.h"
#include "Const.h"

static Mahony_t mahony;
AttitudeEstimator_Handle_t attitudeEstimator;

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

		Mahony_Update(&mahony, &imu);

		Attitude_Data_t *attitude = &attitudeEstimator.data;

		Mahony_GetQuaternion(&mahony, attitude->q);
		attitude->q0 = attitude->q[0];
		attitude->q1 = attitude->q[1];
		attitude->q2 = attitude->q[2];
		attitude->q3 = attitude->q[3];

		Mahony_GetEuler(&mahony, &attitude->roll, &attitude->pitch, &attitude->yaw);

        attitude->rollRate  = imu.gyro.x;
        attitude->pitchRate = imu.gyro.y;
        attitude->yawRate   = imu.gyro.z;

		attitude->biasX = mahony.bias[0];
		attitude->biasY = mahony.bias[1];
		attitude->biasZ = mahony.bias[2];

        attitude->dt = imu.dt;
		attitude->timestamp_us = imu.timestamp_us;

		AttitudeTopic_Publish(attitude);
    }
}
