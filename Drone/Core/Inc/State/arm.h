#ifndef INC_STATE_ARM_H_
#define INC_STATE_ARM_H_

typedef enum {
	DISARMED = 0,
	ARMING,
	ARMED,
	FAILSAFE
} arm_state_t;

extern volatile arm_state_t arm_state;

void Arm_Init(void);
void Arm_Set(arm_state_t s);   /* bench: call from debugger; RC drives it later */

#endif /* INC_STATE_ARM_H_ */
