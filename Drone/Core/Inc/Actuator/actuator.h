#ifndef INC_ACTUATOR_ACTUATOR_H_
#define INC_ACTUATOR_ACTUATOR_H_

#include <stdint.h>

#define MOTOR_COUNT   4

typedef struct{
    uint8_t attitudeSaturated;     /* 1: Roll/Pitch/Yaw is scale down because of motor output limit */
    uint8_t throttleSaturatedHigh; /* 1: Throttle is over max (MOTOR_MAX) */
    uint8_t throttleSaturatedLow;  /* 1: Throttle is below min (MOTOR_IDLE) */
} MotorSaturation_t;

void MotorOutput_Init(void);
void MotorOutput_Update(float roll, float pitch, float yaw, float throttle, MotorSaturation_t *sat);
void ESC_Calibrate(void);

#endif /* INC_ACTUATOR_ACTUATOR_H_ */
