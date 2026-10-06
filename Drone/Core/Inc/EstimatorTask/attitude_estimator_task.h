#ifndef INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_
#define INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_

#include <attitude_topic.h>
#include "FreeRTOS.h"
#include "task.h"
#include "attitude_estimator_task.h"
#include "mahony.h"
#include "imu_topic.h"
#include "mag_topic.h"
#include "Const.h"
#include <stdio.h>
#include "bmi088.h"
#include "timebase.h"
#include "flight_mode.h"
#include "arm.h"
#include "mahony.h"

/* ANGLE and ALT_HOLD never use the yaw angle (yaw is flown as a rate), so the
 * magnetometer can only hurt them: motor current bends its field right when
 * the throttle comes up. Enable for POS_HOLD, after a proper mag calibration. */
#define ATT_EST_USE_MAG  (FlightMode_GetActive() == FLIGHT_MODE_POS_HOLD)

typedef struct {
	Attitude_Data_t data;
	TaskHandle_t attitudeTask;
} AttitudeEstimator_Handle_t;

void AttitudeEstimatorTask(void *argument);
void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_ */
