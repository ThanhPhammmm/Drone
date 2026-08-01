#ifndef INC_STATE_ARM_H_
#define INC_STATE_ARM_H_

#include <stdint.h>

typedef enum {
	DISARMED = 0,
	ARMING,
	ARMED,
	FAILSAFE
} arm_state_t;

extern volatile arm_state_t arm_state;

void Arm_Init(void);
void Arm_Set(arm_state_t s);
void Arm_Update(uint8_t armRequest, uint8_t throttleIdle,  uint8_t linkOk, uint32_t linkLostMs);
uint8_t Arm_MotorsAllowed(void);

#endif /* INC_STATE_ARM_H_ */
