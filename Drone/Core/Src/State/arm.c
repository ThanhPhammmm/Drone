#include "arm.h"

volatile arm_state_t arm_state = DISARMED;

void Arm_Init(void){
	arm_state = DISARMED;
}

void Arm_Set(arm_state_t s){
	arm_state = s;
}
