#ifndef INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "rate_setpoint_topic.h"
#include "attitude_topic.h"
#include "attitude_setpoint_topic.h"
#include "tilt_topic.h"
#include "timebase.h"
#include "Const.h"
#include "arm.h"

typedef struct {
	RateSetpoint_Data_t data;
	TaskHandle_t controllerTask;
	float rollSp;
	float pitchSp;
} AttitudeController_Handle_t;

void AttitudeControllerTask(void *argument);
void AttitudeController_SetTaskHandle(TaskHandle_t handle);
void AttitudeController_GetAngleSetpoint(float angle[2]);

#endif /* INC_CONTROLLERTASK_ATTITUDE_CONTROLLER_TASK_H_ */
