#include "mahony.h"
#include <math.h>

static inline void normalize3(float *x, float *y,float *z){
    float norm = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    if(norm < 1e-6f) return;

    norm = 1.0f / norm;

    *x *= norm;
    *y *= norm;
    *z *= norm;
}

void Mahony_Init(Mahony_t *m, float kp,float ki){
    m->kp = kp;
    m->ki = ki;

    m->q[0] = 1.0f;
    m->q[1] = 0.0f;
    m->q[2] = 0.0f;
    m->q[3] = 0.0f;

    m->bias[0] = 0.0f;
    m->bias[1] = 0.0f;
    m->bias[2] = 0.0f;
}

void Mahony_Update(Mahony_t *m, BMI088_Data_t *imu){
    normalize3(&imu->accel.x, &imu->accel.y, &imu->accel.z);

    float q0 = m->q[0];
    float q1 = m->q[1];
    float q2 = m->q[2];
    float q3 = m->q[3];

    float vx = 2.0f*(q1*q3 - q0*q2);

    float vy = 2.0f*(q0*q1 + q2*q3);

    float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    float ex = imu->accel.y*vz - imu->accel.z*vy;

    float ey = imu->accel.z*vx - imu->accel.x*vz;

    float ez = imu->accel.x*vy - imu->accel.y*vx;

    m->bias[0] += m->ki * ex * imu->dt;
    m->bias[1] += m->ki * ey * imu->dt;
    m->bias[2] += m->ki * ez * imu->dt;

    imu->gyro.x += m->bias[0];
    imu->gyro.y += m->bias[1];
    imu->gyro.z += m->bias[2];

    imu->gyro.x += m->kp * ex;
    imu->gyro.y += m->kp * ey;
    imu->gyro.z += m->kp * ez;

    float dq0 = 0.5f * (-q1*imu->gyro.x - q2*imu->gyro.y - q3*imu->gyro.z);

    float dq1 = 0.5f * ( q0*imu->gyro.x + q2*imu->gyro.z - q3*imu->gyro.y);

    float dq2 = 0.5f * ( q0*imu->gyro.y - q1*imu->gyro.z + q3*imu->gyro.x);

    float dq3 = 0.5f * ( q0*imu->gyro.z + q1*imu->gyro.y - q2*imu->gyro.x);

    q0 += dq0 * imu->dt;
    q1 += dq1 * imu->dt;
    q2 += dq2 * imu->dt;
    q3 += dq3 * imu->dt;

    float norm = 1.0f / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    m->q[0] = q0 * norm;
    m->q[1] = q1 * norm;
    m->q[2] = q2 * norm;
    m->q[3] = q3 * norm;
}

void Mahony_GetQuaternion(const Mahony_t *m, float q[4]){
    q[0]=m->q[0];
    q[1]=m->q[1];
    q[2]=m->q[2];
    q[3]=m->q[3];
}

void Mahony_GetEuler(const Mahony_t *m, float *roll, float *pitch, float *yaw){
    float q0=m->q[0];
    float q1=m->q[1];
    float q2=m->q[2];
    float q3=m->q[3];

    *roll = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2));

    float s = 2.0f*(q0*q2-q3*q1);

    if(s>1.0f) s=1.0f;

    if(s<-1.0f) s=-1.0f;

    *pitch = asinf(s);

    *yaw = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3));
}
