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

typedef enum{
	POS_PHASE_HOLD = 0,     /* stay at the target */
	POS_PHASE_MOVE,         /* stick out: fly the stick's speed */
	POS_PHASE_BRAKE         /* stick back to centre: stop first, then hold where it stopped */
} PosPhase_t;

typedef struct{
	float posKp;
	float velKp;
	float velKi;
	float iMax;
} PosGains_t;

typedef struct{
	uint8_t    engaged;
	PosPhase_t phase;
	float      brakeTime;   /* s */
	float      iAccelX;     /* m/s^2, the I-term (kept as an output, so a gain change does not make it jump) */
	float      iAccelY;
	uint32_t   lastUs;
	uint8_t    resumable;   /* it was holding a spot when POS_HOLD stopped, sticks untouched since */
	float      savedX;      /* m, that spot */
	float      savedY;
	uint32_t   stoppedUs;   /* when POS_HOLD stopped */
	RC_Data_t  rc;
} PosCtrl_t;

extern PositionController_Handle_t positionController;

void PositionControllerTask(void *argument);
void PositionController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_POSITION_CONTROLLER_TASK_H_ */
