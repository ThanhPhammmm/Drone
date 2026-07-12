#ifndef INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_
#define INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_

#include <attitude_topic.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
	Attitude_Data_t data;
	TaskHandle_t attitudeTask;
} AttitudeEstimator_Handle_t;

void AttitudeEstimatorTask(void *argument);
void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_ */
