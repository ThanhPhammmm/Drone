#include "attitude_controller_task.h"
#include "debug.h"

/* ==========================================================
 * 						PID TUNING
 * ==========================================================
 * Kp (Proportional - Present): Reacts to the current error.
 *
 * Ki (Integral - Past):
 * 		-Accumulates past errors over time.
 *		- Why motors spool up on the bench: Ki constantly adds
 *      up tiny sensor noise because the static drone cannot
 *      physically move to correct it (I-term windup).
 *
 * Kd (Derivative - Future): Reacts to the rate of change.
 * 		- Acts as a damper/brake to prevent overshoot.
 * ========================================================== */

#define ATTITUDE_KP_ROLL			6.0f
#define ATTITUDE_KP_PITCH			6.0f

#define RATE_MAX_ROLL				8.0f    /* ~458 deg/s */
#define RATE_MAX_PITCH				8.0f
#define RATE_MAX_YAW				4.0f    /* ~229 deg/s */

#define ATTITUDE_MAX_TILT			0.6f    /* ~34 deg, check rc_task.c accordingly */
#define ATTITUDE_CTRL_RATE_HZ		250
#define ATTITUDE_CTRL_PERIOD_MS		(1000 / ATTITUDE_CTRL_RATE_HZ)
#define ATTITUDE_TILT_MAX_AGE_US	50000   /* position controller output older than this: fly the sticks */
#define ATTITUDE_SOURCE_SLEW		1.0f    /* rad/s: after the tilt source changes (sticks <-> position
                                             * controller), the angle moves to the new one this fast
                                             * instead of jumping (17 deg in 0.3 s) */

#define CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

/* v one step of at most `step` toward target */
static float AttitudeController_Approach(float v, float target, float step){
	if(target > v + step) return v + step;
	if(target < v - step) return v - step;
	return target;
}

AttitudeController_Handle_t attitudeController;

void AttitudeController_SetTaskHandle(TaskHandle_t handle){
	attitudeController.controllerTask = handle;
}

void AttitudeController_GetAngleSetpoint(float angle[2]){
	angle[0] = attitudeController.rollSp;
	angle[1] = attitudeController.pitchSp;
}

void AttitudeControllerTask(void *argument){
	AttitudeController_SetTaskHandle(xTaskGetCurrentTaskHandle());

	Attitude_Data_t attitude = {0};
	AttitudeSetpoint_Data_t setpoint = {0};
	Tilt_Data_t tilt = {0};
	Tilt_Data_t tiltNow;

	uint8_t lastUsePosition = 0;
	uint8_t easing          = 0;	/* source just changed: the angle is still moving over */
	float   rollOut         = 0.0f;
	float   pitchOut        = 0.0f;
	const float slewStep    = ATTITUDE_SOURCE_SLEW * (float)ATTITUDE_CTRL_PERIOD_MS * 0.001f;

	TickType_t last = xTaskGetTickCount();

	while(1){
		vTaskDelayUntil(&last, pdMS_TO_TICKS(ATTITUDE_CTRL_PERIOD_MS));

		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;
		if(AttitudeSetpointTopic_Copy(&setpoint) != pdPASS) continue;

		/* POS_HOLD: the position controller sets the tilt, the sticks only its speed */
		if(TiltTopic_Copy(&tiltNow) == pdPASS) tilt = tiltNow;
		int32_t tiltAge  = Time_DiffUs(attitude.timestamp_us, tilt.timestamp_us);
		uint8_t usePosition = tilt.active && tiltAge < ATTITUDE_TILT_MAX_AGE_US && tiltAge > -ATTITUDE_TILT_MAX_AGE_US;

		RateSetpoint_Data_t* rate = &attitudeController.data;

		if(!Arm_MotorsAllowed()){
			rate->rollRate  = 0.0f;
			rate->pitchRate = 0.0f;
			rate->yawRate   = 0.0f;
			attitudeController.rollSp  = 0.0f;
			attitudeController.pitchSp = 0.0f;
			lastUsePosition = usePosition;
			easing          = 0;
			rollOut         = 0.0f;
			pitchOut        = 0.0f;
		}
		else{
			float rollTarget  = CLAMP(usePosition ? tilt.roll  : setpoint.roll,  -ATTITUDE_MAX_TILT, ATTITUDE_MAX_TILT);
			float pitchTarget = CLAMP(usePosition ? tilt.pitch : setpoint.pitch, -ATTITUDE_MAX_TILT, ATTITUDE_MAX_TILT);

			/* Entering or leaving POS_HOLD: sticks and position controller may ask
			 * for very different angles. Move over to the new one, do not jump. */
			if(usePosition != lastUsePosition){
				lastUsePosition = usePosition;
				easing          = 1;
			}
			if(easing){
				rollOut  = AttitudeController_Approach(rollOut,  rollTarget,  slewStep);
				pitchOut = AttitudeController_Approach(pitchOut, pitchTarget, slewStep);
				if(rollOut == rollTarget && pitchOut == pitchTarget) easing = 0;
			}
			else{
				rollOut  = rollTarget;
				pitchOut = pitchTarget;
			}
			float rollCmd  = rollOut;
			float pitchCmd = pitchOut;
			attitudeController.rollSp  = rollCmd;
			attitudeController.pitchSp = pitchCmd;

			/* P controller: attitude error -> desired body rate */
			rate->rollRate  = ATTITUDE_KP_ROLL  * (rollCmd  - attitude.roll);
			rate->pitchRate = ATTITUDE_KP_PITCH * (pitchCmd - attitude.pitch);
			rate->yawRate   = setpoint.yawRate;   /* yaw commanded as rate directly */

			/* Rate Limiter */
			rate->rollRate  = CLAMP(rate->rollRate,  -RATE_MAX_ROLL,  RATE_MAX_ROLL);
			rate->pitchRate = CLAMP(rate->pitchRate, -RATE_MAX_PITCH, RATE_MAX_PITCH);
			rate->yawRate   = CLAMP(rate->yawRate,   -RATE_MAX_YAW,   RATE_MAX_YAW);
		}

		rate->timestamp_us = attitude.timestamp_us;

		RateSetpointTopic_Publish(rate);
	}
}
