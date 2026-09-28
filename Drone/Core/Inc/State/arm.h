#ifndef INC_STATE_ARM_H_
#define INC_STATE_ARM_H_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "pid.h"

typedef enum {
	DISARMED = 0,
	ARMING,
	ARMED_GROUND,
	FLIGHT_TAKEOFF,
	AIRBORNE,
	LANDING,
	FAILSAFE
} arm_state_t;

extern volatile arm_state_t arm_state;

void Arm_Init(void);
void Arm_Set(arm_state_t s);
void Arm_Update(uint8_t armRequest, uint8_t throttleIdle,  uint8_t linkOk, uint32_t linkLostMs, float thrust,
		float altitudeM, float verticalSpeed, uint8_t altitudeValid);
uint8_t Arm_MotorsAllowed(void);
uint8_t Arm_IntegratorHold(void);
arm_state_t Arm_GetState();

#endif /* INC_STATE_ARM_H_ */
