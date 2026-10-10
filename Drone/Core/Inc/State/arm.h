#ifndef INC_STATE_ARM_H_
#define INC_STATE_ARM_H_

#include <stdint.h>
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"
#include "pid.h"

/* 1: props spin at MOTOR_IDLE as soon as the aircraft is armed (ARMED_GROUND).
 * 0: props stay stopped until take-off. */
#define ARM_SPIN_AT_GROUND_IDLE		1

typedef enum {
	DISARMED = 0,
	ARMING,				/* arm switch on, waiting ARM_CONFIRM_MS; motors stopped */
	ARMED_GROUND,		/* armed on the ground; motors at idle, controllers held in reset */
	FLIGHT_TAKEOFF,		/* leaving the ground; controllers run, no I-term, no air-mode */
	AIRBORNE,			/* flying; full authority */
	LANDING,			/* flying, pilot descending; land detector armed */
	FAILSAFE			/* link lost in flight; level + descend, disarm once landed */
} arm_state_t;

typedef struct{
	uint8_t  armSwitch;
	uint8_t  sensorsReady;		/* IMU / mag / baro calibration finished */
	uint8_t  linkOk;
	uint32_t linkLostMs;
	uint8_t  throttleLow;		/* throttle stick at the bottom (required to arm) */
	uint8_t  takeoffRequest;
	uint8_t  descendRequest;
	float    thrust;
	float    hoverThrust;		/* thrust that hovers, measured in flight; 0 = not known yet */
	float    liftVz;			/* m/s up, accelerometer only: how fast it leaves the ground during the take-off */
	float    accelUp;			/* m/s^2 up */
	uint8_t  altitudeValid;
	float    cosTilt;			/* cos of the angle between body z and vertical (1 = level, <0 = upside down) */
} Arm_Input_t;

extern volatile arm_state_t arm_state;

void Arm_Init(void);
void Arm_Set(arm_state_t s);
void Arm_Update(const Arm_Input_t *in);
arm_state_t Arm_GetState(void);

uint8_t Arm_MotorsAllowed(void);
uint8_t Arm_GroundIdle(void);
uint8_t Arm_IsAirborne(void);
uint8_t Arm_Tumbled(void);			/* 1: the last disarm was the tumble cut-off */

#endif /* INC_STATE_ARM_H_ */
