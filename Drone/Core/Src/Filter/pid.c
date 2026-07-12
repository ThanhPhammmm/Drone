#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integralLimit){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->integralLimit = integralLimit;

    pid->prevError = 0.0f;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt){
    float error = setpoint - measurement;

    pid->integral += error * dt;
    if(pid->integral > pid->integralLimit) pid->integral = pid->integralLimit;
    if(pid->integral < -pid->integralLimit) pid->integral = -pid->integralLimit;

    float derivative = (dt > 0.0f) ? (error - pid->prevError) / dt : 0.0f;
    pid->prevError = error;

    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
