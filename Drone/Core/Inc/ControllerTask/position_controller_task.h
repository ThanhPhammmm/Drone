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

	float targetX;          /* m, level frame: the point it follows. The sticks move it, centred it stands still */
	float targetY;
	float vxSetpoint;       /* m/s, level frame */
	float vySetpoint;
} PositionController_Handle_t;

typedef struct{
	float posKp;
	float velKp;
	float velKi;
	float iMax;
} PosGains_t;

typedef struct{
	uint8_t    engaged;
	float      vTargetX;    /* m/s, level frame: the speed the target moves at now (0 = it stands still) */
	float      vTargetY;
	uint8_t    stopping;    /* the target moved and has stopped, the drone is not at rest yet: the target waits on it */
	float      stopTime;    /* s, for how long */
	float      iAccelX;     /* m/s^2, the I-term (kept as an output, so a gain change does not make it jump) */
	float      iAccelY;
	uint32_t   lastUs;
	uint8_t    resumable;   /* savedX/Y is a spot to go back to: sticks untouched since it was saved */
	float      savedX;      /* m, that spot: where it stands on the ground, or the target when POS_HOLD stopped */
	float      savedY;
	uint32_t   stoppedUs;   /* when it was saved: the last moment on the ground, or when POS_HOLD stopped */
	RC_Data_t  rc;
} PosCtrl_t;

extern PositionController_Handle_t positionController;

void PositionControllerTask(void *argument);
void PositionController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_POSITION_CONTROLLER_TASK_H_ */
