#include "pid.h"

#define PID_TWO_PI      6.28318530718f
#define PID_DT_MAX      0.05f

static inline float pid_clamp(float v, float lo, float hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

void PID_Init(PID_t *pid, float kp, float ki, float kd, float integralLimit, float outputLimit, float dCutoffHz){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integralLimit = integralLimit;
    pid->outputLimit   = outputLimit;
    pid->dCutoffHz     = dCutoffHz;

    PID_Reset(pid);
}

void PID_SetGains(PID_t *pid, float kp, float ki, float kd){
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void PID_Reset(PID_t *pid){
    pid->integral        = 0.0f;
    pid->prevMeasurement = 0.0f;
    pid->dFiltered       = 0.0f;
    pid->output          = 0.0f;
    pid->seeded          = 0;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt){
	if(!(dt > 0.0f) || dt > PID_DT_MAX) dt = 0.0f;

    float error = setpoint - measurement;
    float derivative = 0.0f;
    if(dt > 0.0f && pid->seeded){
        derivative = -(measurement - pid->prevMeasurement) / dt;
    }
    pid->prevMeasurement = measurement;
    pid->seeded          = 1;

    if(pid->dCutoffHz > 0.0f && dt > 0.0f){
        float rc    = 1.0f / (PID_TWO_PI * pid->dCutoffHz);
        float alpha = dt / (rc + dt);
        pid->dFiltered += alpha * (derivative - pid->dFiltered);
    }
    else{
        pid->dFiltered = derivative;
    }

    float integral = pid->integral;
    if(dt > 0.0f && pid->ki != 0.0f){
        integral += error * dt;
        integral  = pid_clamp(integral, -pid->integralLimit, pid->integralLimit);
    }

    float out = pid->kp * error + pid->ki * integral + pid->kd * pid->dFiltered;

    if(pid->outputLimit > 0.0f){
        float clamped = pid_clamp(out, -pid->outputLimit, pid->outputLimit);
        if(clamped != out && (error * out) > 0.0f){
            integral = pid->integral;
            out = pid->kp * error + pid->ki * integral + pid->kd * pid->dFiltered;
            clamped = pid_clamp(out, -pid->outputLimit, pid->outputLimit);
        }
        out = clamped;
    }

    pid->integral = integral;
    pid->output   = out;

    return out;
}
