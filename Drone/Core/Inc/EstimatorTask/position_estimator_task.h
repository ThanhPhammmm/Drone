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

extern PositionEstimator_Handle_t positionEstimator;

void PositionEstimatorTask(void *argument);
void PositionEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_POSITION_ESTIMATOR_TASK_H_ */
