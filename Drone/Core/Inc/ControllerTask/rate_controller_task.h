#ifndef INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "rate_setpoint_topic.h"
#include "pid.h"
#include "Const.h"
#include "attitude_topic.h"
#include "motor_output.h"
#include "arm.h"
#include <stm32f4xx_hal.h>
#include <stdio.h>
#include "thrust_topic.h"

typedef struct {
	float rollTorqueOutput;
	float pitchTorqueOutput;
	float yawTorqueOutput;

	TaskHandle_t controllerTask;
} RateController_Handle_t;

void RateControllerTask(void *argument);
void RateController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_RATE_CONTROLLER_TASK_H_ */
