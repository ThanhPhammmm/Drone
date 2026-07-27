#ifndef INC_FILTER_PID_PID_H_
#define INC_FILTER_PID_PID_H_

#include <stdint.h>

typedef struct{
    float kp;
    float ki;
    float kd;

    float integralLimit;
    float outputLimit;
    float dCutoffHz;
    float integral;
    float prevMeasurement;
    float dFiltered;
    float output;

    uint8_t seeded;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float integralLimit, float outputLimit, float dCutoffHz);
void  PID_SetGains(PID_t *pid, float kp, float ki, float kd);
void  PID_Reset(PID_t *pid);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#endif /* INC_FILTER_PID_PID_H_ */
