#ifndef INC_FILTER_PID_PID_H_
#define INC_FILTER_PID_PID_H_

typedef struct{
    float kp;
    float ki;
    float kd;

    float integral;
    float integralLimit;

    float prevError;
} PID_t;

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integralLimit);
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

#endif /* INC_FILTER_PID_PID_H_ */
