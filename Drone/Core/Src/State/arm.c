#include "arm.h"

#define ARM_CONFIRM_MS				500U    /* hold the switch this long to arm */

#define LIFTOFF_VZ					0.25f   /* m/s up = off the ground */
#define LIFTOFF_CONFIRM_MS			100U
#define LIFTOFF_GROUND_VZ			0.10f   /* aborted take-off below this: it never left the ground */

#define LANDING_HOVER_FRACTION		0.6f    /* thrust at most this part of the hover thrust... */
#define LANDING_THRUST_MAX			0.12f   /* ...or this, if lower or no hover thrust is known yet */
#define LANDING_ACCEL_MAX			1.0f    /* m/s^2: "does not drop" */
#define LANDING_ACCEL_LPF			0.3f    /* per update (50 Hz), ~50 ms */
#define LANDING_CONFIRM_MS			300U

#define FAILSAFE_BLIND_DISARM_MS	5000U   /* link lost, no altitude estimate -> cut motors */
#define FAILSAFE_MAX_DISARM_MS		30000U  /* link lost, descending on the altitude estimate -> backstop */

#define ARM_MAX_TILT_COS			0.906f  /* cos 25 deg */

#define TUMBLE_TILT_COS				0.5f    /* cos 60 deg */
#define TUMBLE_CONFIRM_MS			200U

typedef struct{
	uint8_t  running;
	uint32_t since;
} Arm_Timer_t;

volatile arm_state_t arm_state = DISARMED;

static uint8_t     prevArmSwitch		= 1;
static uint32_t    armingTick			= 0;
static float       accelUpFiltered		= 0.0f;
static Arm_Timer_t liftoffTimer;
static Arm_Timer_t landedTimer;
static Arm_Timer_t tumbleTimer;
static uint8_t     tumbled				= 0;

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
	arm_state            = s;
	liftoffTimer.running = 0;
	landedTimer.running  = 0;
}

static uint8_t Arm_LiftoffConfirmed(const Arm_Input_t *in, uint32_t now){
	return Arm_Hold(&liftoffTimer, in->liftVz >= LIFTOFF_VZ, now, LIFTOFF_CONFIRM_MS);
}

static uint8_t Arm_Landed(const Arm_Input_t *in, uint32_t now){
	float limit = LANDING_HOVER_FRACTION * in->hoverThrust;
	if(limit < LANDING_THRUST_MAX) limit = LANDING_THRUST_MAX;

	uint8_t thrustLow   = in->thrust <= limit;
	uint8_t notDropping = fabsf(accelUpFiltered) <= LANDING_ACCEL_MAX;
	return Arm_Hold(&landedTimer, thrustLow && notDropping, now, LANDING_CONFIRM_MS);
}

void Arm_Init(void){
	prevArmSwitch		= 1;	/* a switch left on at power-up must be cycled before arming */
	armingTick			= 0;
	accelUpFiltered		= 0.0f;
	tumbled				= 0;
	tumbleTimer.running	= 0;
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

uint8_t Arm_Tumbled(void){
	return tumbled;
}

uint8_t Arm_IsAirborne(void){
	return (arm_state == AIRBORNE || arm_state == LANDING || arm_state == FAILSAFE);
}

void Arm_Update(const Arm_Input_t *in){
	uint32_t now = HAL_GetTick();

	uint8_t armEdge = in->armSwitch && !prevArmSwitch;
	prevArmSwitch   = in->armSwitch;
	uint8_t level   = in->cosTilt >= ARM_MAX_TILT_COS;

	accelUpFiltered += (in->accelUp - accelUpFiltered) * LANDING_ACCEL_LPF;

	uint8_t tipping = Arm_MotorsAllowed() && in->cosTilt < TUMBLE_TILT_COS;
	if(Arm_Hold(&tumbleTimer, tipping, now, TUMBLE_CONFIRM_MS)){
		tumbled = 1;
		Arm_Enter(DISARMED);
		return;
	}

	switch(arm_state){

	case DISARMED:
		if(armEdge && in->linkOk && in->sensorsReady && in->throttleLow && level){
			tumbled = 0;
			Arm_Enter(ARMING);
			armingTick = now;
		}
		break;

	case ARMING:
		if(!in->armSwitch || !in->linkOk || !in->sensorsReady || !in->throttleLow || !level){
			Arm_Enter(DISARMED);
		}
		else if((now - armingTick) >= ARM_CONFIRM_MS){
			Arm_Enter(ARMED_GROUND);
		}
		break;

	case ARMED_GROUND:
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
			Arm_Enter((in->liftVz < LIFTOFF_GROUND_VZ) ? ARMED_GROUND : LANDING);
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
