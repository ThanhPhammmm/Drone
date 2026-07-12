#include "attitude_controller_task.h"
#include "attitude_topic.h"
#include "attitude_setpoint_topic.h"
#include "Const.h"

#define ATTITUDE_KP_ROLL			8.0f
#define ATTITUDE_KP_PITCH			8.0f

#define RATE_MAX_ROLL				8.0f    /* ~458 deg/s */
#define RATE_MAX_PITCH				8.0f
#define RATE_MAX_YAW				4.0f    /* ~229 deg/s */

#define ATTITUDE_CTRL_PERIOD_MS		4       /* 250 Hz */

#define CLAMP(v, lo, hi)  ((v) < (lo) ? (lo) : ((v) > (hi) ? (hi) : (v)))

AttitudeController_Handle_t attitudeController;

void AttitudeController_SetTaskHandle(TaskHandle_t handle){
	attitudeController.controllerTask = handle;
}

void AttitudeControllerTask(void *argument){
	AttitudeController_SetTaskHandle(xTaskGetCurrentTaskHandle());

	Attitude_Data_t attitude;
	AttitudeSetpoint_Data_t setpoint;

	TickType_t last = xTaskGetTickCount();

	while(1){
		vTaskDelayUntil(&last, pdMS_TO_TICKS(ATTITUDE_CTRL_PERIOD_MS));

		AttitudeTopic_Copy(&attitude);
		AttitudeSetpointTopic_Copy(&setpoint);

		RateSetpoint_Data_t* rate = &attitudeController.data;

		/* P controller: attitude error -> desired body rate */
		rate->rollRate  = ATTITUDE_KP_ROLL  * (setpoint.roll  - attitude.roll);
		rate->pitchRate = ATTITUDE_KP_PITCH * (setpoint.pitch - attitude.pitch);
		rate->yawRate   = setpoint.yawRate;   /* yaw commanded as rate directly */

		/* Rate Limiter */
		rate->rollRate  = CLAMP(rate->rollRate,  -RATE_MAX_ROLL,  RATE_MAX_ROLL);
		rate->pitchRate = CLAMP(rate->pitchRate, -RATE_MAX_PITCH, RATE_MAX_PITCH);
		rate->yawRate   = CLAMP(rate->yawRate,   -RATE_MAX_YAW,   RATE_MAX_YAW);

		rate->timestamp_us = attitude.timestamp_us;

		RateSetpointTopic_Publish(rate);
	}
}
