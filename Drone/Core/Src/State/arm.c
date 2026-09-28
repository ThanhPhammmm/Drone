#include "arm.h"

#define ARM_CONFIRM_MS				500     /* hold the switch this long to arm */
#define ARM_FAILSAFE_DISARM_MS		5000    /* link lost this long -> cut motors */

#define GROUND_IDLE_THROTTLE		0.2f

#define TAKEOFF_ALT_THRESHOLD_M		0.30f
#define TAKEOFF_CONFIRM_MS			500U

#define LANDING_ALT_THRESHOLD_M		0.15f
#define LANDING_VSPEED_THRESHOLD	0.3f    /* m/s */
#define LANDING_CONFIRM_MS			300U

#define TAKEOFF_FALLBACK_THROTTLE	0.30f
#define TAKEOFF_FALLBACK_MS			500U
#define LANDING_FALLBACK_MS			1000U

volatile arm_state_t arm_state = DISARMED;

static uint8_t  prevArmRequest		= 1;
static uint32_t armingTick			= 0;
static uint32_t phaseTick			= 0;
static float    groundRefAltitude	= 0.0f;
static uint8_t  groundRefValid		= 0;

void Arm_Init(void){
    arm_state			= DISARMED;
    prevArmRequest		= 1;
    armingTick			= 0;
	phaseTick			= 0;
	groundRefAltitude	= 0.0f;
	groundRefValid		= 0;
}

void Arm_Set(arm_state_t s){ arm_state = s; }

uint8_t Arm_MotorsAllowed(void){
	return (arm_state == AIRBORNE || arm_state == LANDING || arm_state == FAILSAFE || arm_state == FLIGHT_TAKEOFF);
}

uint8_t Arm_IntegratorHold(void){
	return (arm_state == ARMING || arm_state == ARMED_GROUND );
}

void Arm_Update(uint8_t armRequest, uint8_t throttleIdle, uint8_t linkOk,
                 uint32_t linkLostMs, float thrust,
                 float altitudeM, float verticalSpeed, uint8_t altitudeValid){

    uint32_t now = HAL_GetTick();

    switch(arm_state){

    case DISARMED:
        if(linkOk && armRequest && !prevArmRequest && throttleIdle){
            arm_state  = ARMING;
            armingTick = now;
        }
        break;

    case ARMING:
        if(!linkOk || !armRequest || !throttleIdle){
            arm_state = DISARMED;
        }
        else if((now - armingTick) >= ARM_CONFIRM_MS){
            arm_state         = ARMED_GROUND;
            groundRefAltitude = altitudeM;
            groundRefValid    = altitudeValid;
            phaseTick         = 0;
        }
        break;

    case ARMED_GROUND:
        if(altitudeValid){
            groundRefAltitude = altitudeM;
            groundRefValid    = 1;
        }

        if(!linkOk){
            arm_state = DISARMED;
        }
        else if(!armRequest){
            arm_state = DISARMED;
        }
        else if(thrust > GROUND_IDLE_THROTTLE){
            arm_state = FLIGHT_TAKEOFF;
            phaseTick = 0;
        }
        break;

    case FLIGHT_TAKEOFF:
        if(!linkOk){
            arm_state = FAILSAFE;
        }
        else if(!armRequest){
            arm_state = DISARMED;
        }
        else if(thrust <= GROUND_IDLE_THROTTLE){
            arm_state = ARMED_GROUND;
        }
        else if(altitudeValid && groundRefValid){
            if((altitudeM - groundRefAltitude) >= TAKEOFF_ALT_THRESHOLD_M){
                if(phaseTick == 0) phaseTick = now;
                else if((now - phaseTick) >= TAKEOFF_CONFIRM_MS) arm_state = AIRBORNE;
            }
            else{
                phaseTick = 0;
            }
        }
        break;

    case AIRBORNE:
        if(!linkOk){
            arm_state = FAILSAFE;
        }
        else if(!armRequest){
            arm_state = DISARMED;
        }
        else if(thrust <= GROUND_IDLE_THROTTLE){
            arm_state = LANDING;
            phaseTick = 0;
        }
        break;

    case LANDING:
        if(!linkOk){
            arm_state = FAILSAFE;
        }
        else if(!armRequest){
            arm_state = DISARMED;
        }
        else if(thrust > GROUND_IDLE_THROTTLE){
            arm_state = AIRBORNE;
            phaseTick = 0;
        }
        else if(altitudeValid && groundRefValid){
            uint8_t nearGround = fabsf(altitudeM - groundRefAltitude) <= LANDING_ALT_THRESHOLD_M;
            uint8_t settled    = fabsf(verticalSpeed) <= LANDING_VSPEED_THRESHOLD;
            if(nearGround && settled){
                if(phaseTick == 0) phaseTick = now;
                else if((now - phaseTick) >= LANDING_CONFIRM_MS) arm_state = ARMED_GROUND;
            }
            else{
                phaseTick = 0;
            }
        }
        break;

    case FAILSAFE:
        if(linkOk){
            arm_state = armRequest ? AIRBORNE : DISARMED;
            phaseTick = 0;
        }
        else if(linkLostMs > ARM_FAILSAFE_DISARM_MS){
            arm_state = DISARMED;
        }
        break;
    }

    prevArmRequest = armRequest;
}

arm_state_t Arm_GetState(){
	return arm_state;
}
