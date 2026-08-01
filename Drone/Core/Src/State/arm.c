#include "arm.h"
#include "main.h"

#define ARM_CONFIRM_MS          500     /* hold the switch this long to arm */
#define ARM_FAILSAFE_DISARM_MS  5000    /* link lost this long -> cut motors */

volatile arm_state_t arm_state = DISARMED;

static uint8_t  prevArmRequest = 1;
static uint32_t armingTick     = 0;

void Arm_Init(void){
    arm_state      = DISARMED;
    prevArmRequest = 1;
}

void Arm_Set(arm_state_t s){ arm_state = s; }

uint8_t Arm_MotorsAllowed(void){
    return (arm_state == ARMED || arm_state == FAILSAFE);
}

void Arm_Update(uint8_t armRequest, uint8_t throttleIdle, uint8_t linkOk, uint32_t linkLostMs){

    switch(arm_state){

    case DISARMED:
        if(linkOk && armRequest && !prevArmRequest && throttleIdle){
            arm_state  = ARMING;
            armingTick = HAL_GetTick();
        }
        break;

    case ARMING:
        if(!linkOk || !armRequest || !throttleIdle){
            arm_state = DISARMED;
        }
        else if((HAL_GetTick() - armingTick) >= ARM_CONFIRM_MS){
            arm_state = ARMED;
        }
        break;

    case ARMED:
        if(!armRequest)   arm_state = DISARMED;     /* disarm is immediate */
        else if(!linkOk)  arm_state = FAILSAFE;
        break;

    case FAILSAFE:
        if(linkOk){
            arm_state = armRequest ? ARMED : DISARMED;
        }
        else if(linkLostMs > ARM_FAILSAFE_DISARM_MS){
            arm_state = DISARMED;
        }
        break;
    }

    prevArmRequest = armRequest;
}
