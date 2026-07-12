#include "rate_controller_task.h"
#include "rate_setpoint_topic.h"
#include "pid.h"
#include "Const.h"
#include "attitude_topic.h"
#include "motor_output.h"

#define RATE_PID_KP_ROLL		0.1f
#define RATE_PID_KI_ROLL		0.0f
#define RATE_PID_KD_ROLL		0.002f

#define RATE_PID_KP_PITCH		0.1f
#define RATE_PID_KI_PITCH		0.0f
#define RATE_PID_KD_PITCH		0.002f

#define RATE_PID_KP_YAW			0.5f
#define RATE_PID_KI_YAW			0.03f
#define RATE_PID_KD_YAW			0.021f

#define RATE_PID_INTEGRAL_LIMIT	3.0f

RateController_Handle_t rateController;
volatile float g_throttle = 0.0f;

static PID_t rollRatePID;
static PID_t pitchRatePID;
static PID_t yawRatePID;

void RateController_SetTaskHandle(TaskHandle_t handle){
	rateController.controllerTask = handle;
}

void RateControllerTask(void *argument){
	RateController_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(rateController.controllerTask, RATE_CONTROLLER_ID_TASK);

	PID_Init(&rollRatePID,  RATE_PID_KP_ROLL,  RATE_PID_KI_ROLL,  RATE_PID_KD_ROLL,  RATE_PID_INTEGRAL_LIMIT);
	PID_Init(&pitchRatePID, RATE_PID_KP_PITCH, RATE_PID_KI_PITCH, RATE_PID_KD_PITCH, RATE_PID_INTEGRAL_LIMIT);
	PID_Init(&yawRatePID,   RATE_PID_KP_YAW,   RATE_PID_KI_YAW,   RATE_PID_KD_YAW,   RATE_PID_INTEGRAL_LIMIT);

	Attitude_Data_t attitude;
	RateSetpoint_Data_t setpoint;

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          /* 1 kHz, from estimator */
		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;
		RateSetpointTopic_Copy(&setpoint);                /* latest (250 Hz) */

		rateController.rollOutput  = PID_Update(&rollRatePID,  setpoint.rollRate,  attitude.rollRate,  attitude.dt);
		rateController.pitchOutput = PID_Update(&pitchRatePID, setpoint.pitchRate, attitude.pitchRate, attitude.dt);
		rateController.yawOutput   = PID_Update(&yawRatePID,   setpoint.yawRate,   attitude.yawRate,   attitude.dt);

		MotorOutput_Update(rateController.rollOutput, rateController.pitchOutput, rateController.yawOutput, g_throttle);
	}
}
