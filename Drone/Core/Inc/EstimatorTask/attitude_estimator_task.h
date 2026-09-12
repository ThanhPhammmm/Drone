#ifndef INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_
#define INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_

#include <attitude_topic.h>
#include "FreeRTOS.h"
#include "task.h"
#include "attitude_estimator_task.h"
#include "mahony.h"
#include "imu_topic.h"
#include "mag_topic.h"
#include "Const.h"
#include <stdio.h>
#include "bmi088.h"

typedef struct {
	Attitude_Data_t data;
	TaskHandle_t attitudeTask;
} AttitudeEstimator_Handle_t;

void AttitudeEstimatorTask(void *argument);
void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_ */
