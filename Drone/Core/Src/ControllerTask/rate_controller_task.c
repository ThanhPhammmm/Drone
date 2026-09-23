#include "rate_controller_task.h"
#include "debug.h"

#define RATE_PID_KP_ROLL		0.31610f
#define RATE_PID_KI_ROLL		0.01132f
#define RATE_PID_KD_ROLL		0.00510f

#define RATE_PID_KP_PITCH 		0.31610f
#define RATE_PID_KI_PITCH 		0.01132f
#define RATE_PID_KD_PITCH 		0.00510f

#define RATE_PID_KP_YAW			1.73369f
#define RATE_PID_KI_YAW   		0.15271f
#define RATE_PID_KD_YAW   		0.00252f

#define RATE_PID_INTEGRAL_LIMIT	3.0f
#define RATE_PID_OUTPUT_LIMIT	0.4f
#define RATE_PID_D_CUTOFF_HZ	40.0f
#define RATE_SETPOINT_MAX_AGE_US	16000U

RateController_Handle_t rateController;
volatile float g_throttle = 0.0f;
Thrust_Data_t thrust;
static float lastThrust = 0.0f;

static PID_t rollRatePID;
static PID_t pitchRatePID;
static PID_t yawRatePID;

void RateController_SetTaskHandle(TaskHandle_t handle){
	rateController.controllerTask = handle;
}

static void RateController_Idle(void){
	PID_Reset(&rollRatePID);
	PID_Reset(&pitchRatePID);
	PID_Reset(&yawRatePID);

	rateController.rollTorqueOutput  = 0.0f;
	rateController.pitchTorqueOutput = 0.0f;
	rateController.yawTorqueOutput   = 0.0f;

	MotorOutput_Update(0.0f, 0.0f, 0.0f, 0.0f);
}

void RateControllerTask(void *argument){
	RateController_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(rateController.controllerTask, RATE_CONTROLLER_ID_TASK);

	PID_Init(&rollRatePID,  RATE_PID_KP_ROLL,  RATE_PID_KI_ROLL,  RATE_PID_KD_ROLL,
	         RATE_PID_INTEGRAL_LIMIT, RATE_PID_OUTPUT_LIMIT, RATE_PID_D_CUTOFF_HZ);
	PID_Init(&pitchRatePID, RATE_PID_KP_PITCH, RATE_PID_KI_PITCH, RATE_PID_KD_PITCH,
	         RATE_PID_INTEGRAL_LIMIT, RATE_PID_OUTPUT_LIMIT, RATE_PID_D_CUTOFF_HZ);
	PID_Init(&yawRatePID,   RATE_PID_KP_YAW,   RATE_PID_KI_YAW,   RATE_PID_KD_YAW,
	         RATE_PID_INTEGRAL_LIMIT, RATE_PID_OUTPUT_LIMIT, RATE_PID_D_CUTOFF_HZ);

	Attitude_Data_t attitude = {0};
	RateSetpoint_Data_t setpoint = {0};

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          /* 2 kHz, from estimator */
		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;
		if(RateSetpointTopic_Copy(&setpoint) != pdPASS) continue;   /* latest (250 Hz) */

	    if(arm_state != ARMED){
	      RateController_Idle();
	      lastThrust = 0;
	      continue;
	    }

		uint32_t age_us = attitude.timestamp_us - setpoint.timestamp_us;
		if(age_us > RATE_SETPOINT_MAX_AGE_US){
			setpoint.rollRate  = 0.0f;
			setpoint.pitchRate = 0.0f;
			setpoint.yawRate   = 0.0f;
		}
		rateController.rollTorqueOutput  = PID_Update(&rollRatePID,  setpoint.rollRate,  attitude.rollRate,  attitude.dt);
		rateController.pitchTorqueOutput = PID_Update(&pitchRatePID, setpoint.pitchRate, attitude.pitchRate, attitude.dt);
		rateController.yawTorqueOutput   = PID_Update(&yawRatePID,   setpoint.yawRate,   attitude.yawRate,   attitude.dt);

        if(ThrustTopic_Copy(&thrust, 0) == pdPASS){
            lastThrust = thrust.thrust;
        }
		//Motor_Setpoint_Print(&setpoint, lastThrust);
		Motor_Torque_Print(&rateController, lastThrust);
		//MotorOutput_Update(rateController.rollTorqueOutput, rateController.pitchTorqueOutput, rateController.yawTorqueOutput, g_throttle);
        MotorOutput_Update(rateController.rollTorqueOutput, rateController.pitchTorqueOutput, rateController.yawTorqueOutput, lastThrust);
	}
}
