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

/* ==========================================================
 *						COMPASS (QMC5883)
 * ==========================================================
 * Off (0): the heading is the gyro's only and drifts slowly; yaw 0 = the nose
 * at power-up. Nothing flown needs north today (yaw is flown as a rate).
 * On (1): the heading follows the compass, so yaw 0 = north and it stops
 * drifting. It snaps to the compass on the ground (at power-up, and after a
 * compass calibration), then corrects slowly; never jumps in flight. Readings bent
 * by motor current (field strength or dip unlike the ground reference taken with
 * the motors off, or a field that turns with the frame) are skipped, and so are
 * readings while the drone turns fast: the gyro carries the heading meanwhile.
 *
 * Before switching it on, check it on TFT page 3 ("Comp"; it is computed and
 * shown even while off), disarmed, after a compass calibration (page 6) and
 * once "LEARNING" has turned to "OK, NOT USED" (~1 s):
 *	 1. turn the nose to N, E, S, W: Comp reads ~0, 90, 180, 270
 *	 2. tilt the drone 30 deg nose up/down and to each side: Comp moves less than ~5 deg
 *	 3. props off, arm and raise the throttle: Comp stays "OK" and moves less than ~5 deg
 * If 3 fails, motor current reaches the compass: move it away from the power wires.
 * If 1 or 2 fails, the compass axes do not match the IMU's: fix the signs in QMC5883_Read().
 * N, E, S, W read:	 0, 270, 180, 90  -> flip y		   180, 90, 0, 270	-> flip x
 *					 180, 270, 0, 90  -> flip x and y	270, 180, 90, 0	 -> swap x and y
 * right when level but swings when tilted (check 2)   -> flip z */
#define ATT_EST_USE_MAG	1

#define ATT_EST_MAG_DECLINATION_DEG		0.0f	/* magnetic -> true north, east +, west - */
#define ATT_EST_MAG_KP_GROUND			0.2f	/* 1/s: heading correction on the ground (~5 s to settle) */
#define ATT_EST_MAG_KP_FLIGHT			0.1f	/* 1/s: in flight (~10 s): the gyro carries the short term */
#define ATT_EST_MAG_KI_GROUND			0.02f	/* how fast the compass teaches the gyro its yaw drift (~10 s) */
#define ATT_EST_MAG_KI_FLIGHT			0.002f	/* 10x slower in flight, where a bent field is likelier */
#define ATT_EST_MAG_SNAP_DEG			30.0f	/* on the ground, a larger error jumps the heading at once */
#define ATT_EST_MAG_NORM_TOL			0.15f	/* field strength may differ this much (part) from the ground reference */
#define ATT_EST_MAG_DIP_TOL_DEG			8.0f	/* field dip may differ this much from the ground reference */
#define ATT_EST_MAG_REF_SAMPLES			50U		/* readings (~1 s) with the motors off before the reference counts */

typedef struct {
	Attitude_Data_t data;
	TaskHandle_t attitudeTask;
} AttitudeEstimator_Handle_t;

void AttitudeEstimatorTask(void *argument);
void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_ATTITUDE_ESTIMATOR_TASK_H_ */
