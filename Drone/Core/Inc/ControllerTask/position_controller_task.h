#ifndef INC_CONTROLLERTASK_POSITION_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_POSITION_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "position_topic.h"
#include "tilt_topic.h"
#include "rc_topic.h"
#include "Const.h"
#include "arm.h"
#include "flight_mode.h"
#include <math.h>

typedef struct{
	Tilt_Data_t  data;
	TaskHandle_t controllerTask;

	float targetX;          /* m, level frame: where it holds */
	float targetY;
	float vxSetpoint;       /* m/s, level frame */
	float vySetpoint;
} PositionController_Handle_t;

extern PositionController_Handle_t positionController;

void PositionControllerTask(void *argument);
void PositionController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_POSITION_CONTROLLER_TASK_H_ */
