#ifndef INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"

typedef struct {
	float rollOutput;
	float pitchOutput;
	float yawOutput;

	TaskHandle_t controllerTask;
} RateController_Handle_t;

void RateControllerTask(void *argument);
void RateController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_ */
