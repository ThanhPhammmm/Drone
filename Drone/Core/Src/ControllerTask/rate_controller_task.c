#include "rate_controller_task.h"
#include "rate_setpoint_topic.h"
#include "pid.h"
#include "Const.h"
#include "attitude_topic.h"
#include "motor_output.h"
#include "arm.h"
#include <stm32f4xx_hal.h>
#include <stdio.h>
#include "thrust_topic.h"

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
#define RATE_SETPOINT_MAX_AGE_US	100000U

RateController_Handle_t rateController;
volatile float g_throttle = 0.0f;
Thrust_Data_t thrust;
static float lastThrust = 0.0f;

static PID_t rollRatePID;
static PID_t pitchRatePID;
static PID_t yawRatePID;

extern UART_HandleTypeDef huart1;
void BMI088_PrintRate(const RateController_Handle_t* rateController){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f,%.6f,%.6f\r\n",
		rateController->rollTorqueOutput,rateController->pitchTorqueOutput, rateController->yawTorqueOutput);

	if (len > 0 && HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, (uint16_t)len) == HAL_OK){
		last_print_time = current_time;
	}
}

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

	Attitude_Data_t attitude;
	RateSetpoint_Data_t setpoint;

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);          /* 2 kHz, from estimator */
		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;
		if(RateSetpointTopic_Copy(&setpoint) != pdPASS) continue;   /* latest (250 Hz) */

	    if(arm_state != ARMED){
	      RateController_Idle();
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
		//BMI088_PrintRate(&rateController);
		//MotorOutput_Update(rateController.rollTorqueOutput, rateController.pitchTorqueOutput, rateController.yawTorqueOutput, g_throttle);
        MotorOutput_Update(rateController.rollTorqueOutput, rateController.pitchTorqueOutput, rateController.yawTorqueOutput, lastThrust);
	}
}
