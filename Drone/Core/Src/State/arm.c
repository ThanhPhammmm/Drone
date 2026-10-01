#include "arm.h"

#define ARM_CONFIRM_MS				500U    /* hold the switch this long to arm */

#define TAKEOFF_ALT_THRESHOLD_M		0.30f
#define TAKEOFF_CONFIRM_MS			500U
#define TAKEOFF_FALLBACK_THROTTLE	0.40f   /* a bit above hover thrust: this much for this long = flying */
#define TAKEOFF_FALLBACK_MS			500U

#define LANDING_ALT_THRESHOLD_M		0.15f
#define LANDING_VSPEED_THRESHOLD	0.3f    /* m/s */
#define LANDING_THRUST_MAX			0.12f   /* collective at the floor while nothing moves = sitting on the ground */
#define LANDING_CONFIRM_MS			300U
#define LANDING_FALLBACK_MS			1000U

#define FAILSAFE_BLIND_DISARM_MS	5000U   /* link lost, no altitude estimate -> cut motors */
#define FAILSAFE_MAX_DISARM_MS		30000U  /* link lost, descending on the altitude estimate -> backstop */

typedef struct{
	uint8_t  running;
	uint32_t since;
} Arm_Timer_t;

volatile arm_state_t arm_state = DISARMED;

static uint8_t     prevArmSwitch		= 1;
static uint32_t    armingTick			= 0;
static float       groundRefAltitude	= 0.0f;
static uint8_t     groundRefValid		= 0;
static Arm_Timer_t liftoffAltTimer;
static Arm_Timer_t liftoffThrustTimer;
static Arm_Timer_t landedTimer;

static uint8_t Arm_Hold(Arm_Timer_t *t, uint8_t condition, uint32_t now, uint32_t holdMs){
	if(!condition){
		t->running = 0;
		return 0;
	}
	if(!t->running){
		t->running = 1;
		t->since   = now;
	}
	return (now - t->since) >= holdMs;
}

static void Arm_Enter(arm_state_t s){
	arm_state                  = s;
	liftoffAltTimer.running    = 0;
	liftoffThrustTimer.running = 0;
	landedTimer.running        = 0;
}

static uint8_t Arm_LiftoffConfirmed(const Arm_Input_t *in, uint32_t now){
	uint8_t risen = in->altitudeValid && groundRefValid &&
	                (in->altitude - groundRefAltitude) >= TAKEOFF_ALT_THRESHOLD_M;

	/* fallback when the altitude estimate is missing or does not move */
	uint8_t highThrust = !(in->altitudeValid && groundRefValid) &&
	                     in->thrust >= TAKEOFF_FALLBACK_THROTTLE;

	uint8_t byAltitude = Arm_Hold(&liftoffAltTimer,    risen,      now, TAKEOFF_CONFIRM_MS);
	uint8_t byThrust   = Arm_Hold(&liftoffThrustTimer, highThrust, now, TAKEOFF_FALLBACK_MS);
	return byAltitude || byThrust;
}

static uint8_t Arm_Landed(const Arm_Input_t *in, uint32_t now){
	if(in->altitudeValid){
		uint8_t settled    = fabsf(in->verticalSpeed) <= LANDING_VSPEED_THRESHOLD;
		uint8_t nearGround = groundRefValid && (in->altitude - groundRefAltitude) <= LANDING_ALT_THRESHOLD_M;
		uint8_t thrustLow  = in->thrust <= LANDING_THRUST_MAX;
		return Arm_Hold(&landedTimer, settled && (nearGround || thrustLow), now, LANDING_CONFIRM_MS);
	}
	return Arm_Hold(&landedTimer, in->throttleLow, now, LANDING_FALLBACK_MS);
}

void Arm_Init(void){
	prevArmSwitch		= 1;	/* a switch left on at power-up must be cycled before arming */
	armingTick			= 0;
	groundRefAltitude	= 0.0f;
	groundRefValid		= 0;
	Arm_Enter(DISARMED);
}

void Arm_Set(arm_state_t s){ Arm_Enter(s); }

arm_state_t Arm_GetState(void){
	return arm_state;
}

uint8_t Arm_MotorsAllowed(void){
	return (arm_state == FLIGHT_TAKEOFF || arm_state == AIRBORNE || arm_state == LANDING || arm_state == FAILSAFE);
}

uint8_t Arm_GroundIdle(void){
	return ARM_SPIN_AT_GROUND_IDLE && (arm_state == ARMED_GROUND);
}

uint8_t Arm_IsAirborne(void){
	return (arm_state == AIRBORNE || arm_state == LANDING || arm_state == FAILSAFE);
}

void Arm_Update(const Arm_Input_t *in){
	uint32_t now = HAL_GetTick();

	uint8_t armEdge = in->armSwitch && !prevArmSwitch;
	prevArmSwitch   = in->armSwitch;

	switch(arm_state){

	case DISARMED:
		if(armEdge && in->linkOk && in->sensorsReady && in->throttleLow){
			Arm_Enter(ARMING);
			armingTick = now;
		}
		break;

	case ARMING:
		if(!in->armSwitch || !in->linkOk || !in->sensorsReady || !in->throttleLow){
			Arm_Enter(DISARMED);
		}
		else if((now - armingTick) >= ARM_CONFIRM_MS){
			Arm_Enter(ARMED_GROUND);
			groundRefAltitude = in->altitude;
			groundRefValid    = in->altitudeValid;
		}
		break;

	case ARMED_GROUND:
		if(in->altitudeValid){
			groundRefAltitude = in->altitude;
			groundRefValid    = 1;
		}

		if(!in->armSwitch || !in->linkOk){
			Arm_Enter(DISARMED);
		}
		else if(in->takeoffRequest){
			Arm_Enter(FLIGHT_TAKEOFF);
		}
		break;

	case FLIGHT_TAKEOFF:
		if(!in->armSwitch){
			Arm_Enter(DISARMED);
		}
		else if(!in->linkOk){
			Arm_Enter(FAILSAFE);
		}
		else if(in->descendRequest){
			uint8_t stillOnGround = !in->altitudeValid || !groundRefValid ||
			                        (in->altitude - groundRefAltitude) < LANDING_ALT_THRESHOLD_M;
			Arm_Enter(stillOnGround ? ARMED_GROUND : LANDING);
		}
		else if(Arm_LiftoffConfirmed(in, now)){
			Arm_Enter(AIRBORNE);
		}
		break;

	case AIRBORNE:
		if(!in->armSwitch){
			Arm_Enter(DISARMED);
		}
		else if(!in->linkOk){
			Arm_Enter(FAILSAFE);
		}
		else if(in->descendRequest){
			Arm_Enter(LANDING);
		}
		break;

	case LANDING:
		if(!in->armSwitch){
			Arm_Enter(DISARMED);
		}
		else if(!in->linkOk){
			Arm_Enter(FAILSAFE);
		}
		else if(!in->descendRequest){
			Arm_Enter(AIRBORNE);
		}
		else if(Arm_Landed(in, now)){
			Arm_Enter(ARMED_GROUND);
		}
		break;

	case FAILSAFE:
		if(in->linkOk){
			if(!in->armSwitch)          Arm_Enter(DISARMED);
			else if(in->descendRequest) Arm_Enter(LANDING);
			else                        Arm_Enter(AIRBORNE);
		}
		else if(in->altitudeValid){
			if(Arm_Landed(in, now) || in->linkLostMs > FAILSAFE_MAX_DISARM_MS){
				Arm_Enter(DISARMED);
			}
		}
		else if(in->linkLostMs > FAILSAFE_BLIND_DISARM_MS){
			Arm_Enter(DISARMED);
		}
		break;

	default:
		Arm_Enter(DISARMED);
		break;
	}
}
