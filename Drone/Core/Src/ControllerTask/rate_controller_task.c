#include "rate_controller_task.h"
#include "rate_setpoint_topic.h"
#include "pid.h"
#include "Const.h"
#include "attitude_topic.h"
#include "motor_output.h"
#include "arm.h"
#include <stm32f4xx_hal.h>
#include <stdio.h>

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
volatile float g_throttle = 0.5f;

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
		rateController->rollOutput,rateController->pitchOutput, rateController->yawOutput);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

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

	    if(arm_state != ARMED){
	      PID_Reset(&rollRatePID);
	      PID_Reset(&pitchRatePID);
	      PID_Reset(&yawRatePID);

	      rateController.rollOutput  = 0.0f;
	      rateController.pitchOutput = 0.0f;
	      rateController.yawOutput   = 0.0f;

	      MotorOutput_Update(0.0f, 0.0f, 0.0f, 0.0f);
	      continue;
	    }

		rateController.rollOutput  = PID_Update(&rollRatePID,  setpoint.rollRate,  attitude.rollRate,  attitude.dt);
		rateController.pitchOutput = PID_Update(&pitchRatePID, setpoint.pitchRate, attitude.pitchRate, attitude.dt);
		rateController.yawOutput   = PID_Update(&yawRatePID,   setpoint.yawRate,   attitude.yawRate,   attitude.dt);

		//BMI088_PrintRate(&rateController);
		MotorOutput_Update(rateController.rollOutput, rateController.pitchOutput, rateController.yawOutput, g_throttle);
	}
}
