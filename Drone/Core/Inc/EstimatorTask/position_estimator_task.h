#ifndef INC_ESTIMATORTASK_POSITION_ESTIMATOR_TASK_H_
#define INC_ESTIMATORTASK_POSITION_ESTIMATOR_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "attitude_topic.h"
#include "altitude_topic.h"
#include "flow_topic.h"
#include "gps_topic.h"
#include "position_topic.h"
#include "attitude_estimator_task.h"     /* modify ATT_EST_USE_MAG */
#include "position_kalman_filter.h"
#include "pmw3901.h"
#include "Const.h"
#include "timebase.h"
#include "arm.h"
#include <math.h>
#include <string.h>

typedef struct{
	Position_Data_t data;
	TaskHandle_t    positionTask;
} PositionEstimator_Handle_t;

typedef struct{
	uint8_t  haveSample;
	uint32_t lastSampleUs;
	float    dtAcc;
	float    accelXAcc;
	float    accelYAcc;

	float    rollTurn;          /* rad the body rolled since the last flow sample */
	float    pitchTurn;         /* rad it pitched */
	uint32_t lastFlowUs;

	uint8_t  flowPending;       /* a flow velocity waits for the next filter step */
	float    flowVx;            /* m/s, level frame */
	float    flowVy;
	float    flowStd;

	uint16_t usedAgo;           /* filter steps since the filter took a flow velocity */
	uint8_t  rejects;
	uint16_t steps;             /* filter steps so far (stops at POS_EST_HISTORY) */
	Altitude_Data_t alt;        /* last copy: is it standing on the floor? */
} PosEst_t;

/* the estimate at each filter step, for the GPS delay */
typedef struct{
	float x, y;
	float vx, vy;
} PosHist_t;

typedef struct{
	uint32_t lastStampUs;       /* the solution handled last */

	uint8_t  originSet;         /* north/east are measured from the first good fix */
	int32_t  lat0;              /* 1e-7 deg */
	int32_t  lon0;
	float    cosLat0;

	uint8_t  aligned;           /* yawOffset is known */
	float    yawOffset;         /* rad, from north to the level frame's x */
	float    alignC;            /* weighted sum of the angle samples, as unit vectors */
	float    alignS;
	float    alignW;
	uint8_t  alignN;

	float    originX;           /* m, where the GPS origin sits in the level frame */
	float    originY;
	float    x;                 /* m, the last fix in the level frame */
	float    y;

	uint16_t usedAgo;           /* filter steps since the filter took a GPS update */
	uint16_t posUsedAgo;        /* filter steps since it took a GPS position */
	uint8_t  velRejects;
	uint8_t  posRejects;
} PosGps_t;

extern PositionEstimator_Handle_t positionEstimator;

void PositionEstimatorTask(void *argument);
void PositionEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_POSITION_ESTIMATOR_TASK_H_ */
