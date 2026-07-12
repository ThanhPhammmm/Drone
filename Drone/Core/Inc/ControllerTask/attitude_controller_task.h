#ifndef INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "rate_setpoint_topic.h"

typedef struct {
	RateSetpoint_Data_t data;
	TaskHandle_t controllerTask;
} AttitudeController_Handle_t;

void AttitudeControllerTask(void *argument);
void AttitudeController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_ */
