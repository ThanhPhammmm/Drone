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

#define RATE_PID_INTEGRAL_LIMIT			3.0f
#define RATE_PID_OUTPUT_LIMIT			0.4f
#define RATE_PID_D_CUTOFF_HZ			40.0f
#define RATE_SETPOINT_MAX_AGE_US		16000

RateController_Handle_t rateController;
static Thrust_Data_t thrust;
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

	MotorOutput_Update(0.0f, 0.0f, 0.0f, 0.0f, NULL);
}

static void RateController_ResetIntegrators(void){
	PID_ResetIntegral(&rollRatePID);
	PID_ResetIntegral(&pitchRatePID);
	PID_ResetIntegral(&yawRatePID);
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
	uint32_t lastSampleUs = 0;
	uint8_t  haveSample   = 0;

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          /* 1 kHz, from estimator */
		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;

		if(haveSample && attitude.timestamp_us == lastSampleUs) continue;
		float dt = haveSample ? (float)(uint32_t)(attitude.timestamp_us - lastSampleUs) * 1e-6f : 0.0f;
		lastSampleUs = attitude.timestamp_us;
		haveSample   = 1;

		if(RateSetpointTopic_Copy(&setpoint) != pdPASS) continue;   /* latest (250 Hz) */

        if(ThrustTopic_Copy(&thrust, 0) == pdPASS){
            lastThrust = thrust.thrust;
        }

        if(!Arm_MotorsAllowed()){
        	RateController_Idle();
        	continue;
        }

        int32_t age_us = Time_DiffUs(attitude.timestamp_us, setpoint.timestamp_us);
		if(age_us > (int32_t)RATE_SETPOINT_MAX_AGE_US || age_us < -(int32_t)RATE_SETPOINT_MAX_AGE_US){
			setpoint.rollRate  = 0.0f;
			setpoint.pitchRate = 0.0f;
			setpoint.yawRate   = 0.0f;
		}

		/* Integrators only run once off the ground. While the frame is pinned by
		 * the ground the rate error can never be removed, the I-term winds up and
		 * is released as a kick at lift-off. */
		if(!Arm_IsAirborne()){
			RateController_ResetIntegrators();
		}

		rateController.rollTorqueOutput  = PID_Update(&rollRatePID,  setpoint.rollRate,  attitude.rollRate,  dt);
		rateController.pitchTorqueOutput = PID_Update(&pitchRatePID, setpoint.pitchRate, attitude.pitchRate, dt);
		rateController.yawTorqueOutput   = PID_Update(&yawRatePID,   setpoint.yawRate,   attitude.yawRate,   dt);

		MotorSaturation_t sat = {0};
        MotorOutput_Update(rateController.rollTorqueOutput, rateController.pitchTorqueOutput, rateController.yawTorqueOutput, lastThrust, &sat);

        /* Anti-windup: the mixer could not deliver the torque on that axis, so
         * do not let the integrator keep growing. */
        if(sat.rollPitchSaturated){
        	PID_HoldIntegrator(&rollRatePID);
        	PID_HoldIntegrator(&pitchRatePID);
        }
        if(sat.yawSaturated){
        	PID_HoldIntegrator(&yawRatePID);
        }
	}
}

void RateController_GetTorque(float torque[3]){
	torque[0] = rateController.rollTorqueOutput;
	torque[1] = rateController.pitchTorqueOutput;
	torque[2] = rateController.yawTorqueOutput;
}
