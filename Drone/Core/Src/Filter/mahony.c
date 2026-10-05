#include "mahony.h"
#include <math.h>

#define MAHONY_G                9.80665f
#define MAHONY_ACC_TOL          0.20f     /* accept 0.8 g .. 1.2 g */

#define MAHONY_BIAS_LIMIT       0.10f     /* ~5.7 deg/s */

#define MAHONY_DT_MAX           0.05f

/* Earth field is 0.25..0.65 G. Outside this band the reading is a bad
 * calibration or motor/ESC current, not the earth. */
#define MAHONY_MAG_NORM_MIN     0.15f     /* gauss */
#define MAHONY_MAG_NORM_MAX     1.00f     /* gauss */
#define MAHONY_MAG_WEIGHT       0.2f      /* heading error (rad) -> correction, relative to kp */

static inline float mahony_clamp(float v, float lim){
    return v < -lim ? -lim : (v > lim ? lim : v);
}

static inline uint8_t normalize3(float *x, float *y,float *z){
    float norm = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
    if(norm < 1e-6f) return 0;

    norm = 1.0f / norm;

    *x *= norm;
    *y *= norm;
    *z *= norm;

    return 1;
}

/* Rotate the quaternion about the earth vertical by `dyaw` (rad). */
static void mahony_rotate_yaw(Mahony_t *m, float dyaw){
    float c = cosf(0.5f * dyaw);
    float s = sinf(0.5f * dyaw);
    float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

    /* q' = (c, 0, 0, s) * q  -- earth-frame rotation */
    m->q[0] = c*q0 - s*q3;
    m->q[1] = c*q1 - s*q2;
    m->q[2] = c*q2 + s*q1;
    m->q[3] = c*q3 + s*q0;
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

    m->rate[0] = 0.0f;
    m->rate[1] = 0.0f;
    m->rate[2] = 0.0f;
}

void Mahony_Update(Mahony_t *m, const BMI088_Data_t *imu, const float mag[3], uint8_t magValid){
    float ax = imu->accel.x;
    float ay = imu->accel.y;
    float az = imu->accel.z;
    float dt = imu->dt;

    if(!(dt > 0.0f) || dt > MAHONY_DT_MAX){
        m->rate[0] = imu->gyro.x + m->bias[0];
        m->rate[1] = imu->gyro.y + m->bias[1];
        m->rate[2] = imu->gyro.z + m->bias[2];
        return;
    }

    float aNorm = sqrtf(ax*ax + ay*ay + az*az);
    uint8_t accValid = (aNorm > MAHONY_G * (1.0f - MAHONY_ACC_TOL)) && (aNorm < MAHONY_G * (1.0f + MAHONY_ACC_TOL));
    if(accValid) accValid = normalize3(&ax, &ay, &az);

    float q0 = m->q[0];
    float q1 = m->q[1];
    float q2 = m->q[2];
    float q3 = m->q[3];

    float ex = 0.0f;
    float ey = 0.0f;
    float ez = 0.0f;

    float hex = 0.0f;
    float hey = 0.0f;
    float hez = 0.0f;

    // predicted direction of gravity
    float vx = -2.0f*(q1*q3 - q0*q2);
    float vy = -2.0f*(q0*q1 + q2*q3);
    float vz = -(q0*q0 - q1*q1 - q2*q2 + q3*q3);

    if(accValid){
        ex = ay*vz - az*vy;
        ey = az*vx - ax*vz;
        ez = ax*vy - ay*vx;
    }

    float mx = mag[0];
    float my = mag[1];
    float mz = mag[2];
    float mNorm = sqrtf(mx*mx + my*my + mz*mz);
    if(magValid && mNorm > MAHONY_MAG_NORM_MIN && mNorm < MAHONY_MAG_NORM_MAX){
        mx /= mNorm;
        my /= mNorm;
        mz /= mNorm;
        // measured magnetic field → world frame
        float hx = 2.0f*mx*(0.5f - q2*q2 - q3*q3) + 2.0f*my*(q1*q2 - q0*q3) + 2.0f*mz*(q1*q3 + q0*q2);
        float hy = 2.0f*mx*(q1*q2 + q0*q3) + 2.0f*my*(0.5f - q1*q1 - q3*q3) + 2.0f*mz*(q2*q3 - q0*q1);

        if(hx*hx + hy*hy > 1e-6f){
            // heading error, magnetic north = yaw 0 (declination ignored)
            float headingErr = atan2f(hy, hx);

            if(!m->headingAligned){
                /* start from the magnetometer heading instead of converging to it */
                mahony_rotate_yaw(m, -headingErr);
                m->headingAligned = 1;
                q0 = m->q[0]; q1 = m->q[1]; q2 = m->q[2]; q3 = m->q[3];
                headingErr = 0.0f;
            }
            headingErr *= MAHONY_MAG_WEIGHT;
            hex = headingErr * vx;
            hey = headingErr * vy;
            hez = headingErr * vz;
        }
    }

    m->bias[0] = mahony_clamp(m->bias[0] + m->ki * ex * dt, MAHONY_BIAS_LIMIT);
    m->bias[1] = mahony_clamp(m->bias[1] + m->ki * ey * dt, MAHONY_BIAS_LIMIT);
    m->bias[2] = mahony_clamp(m->bias[2] + m->ki * ez * dt, MAHONY_BIAS_LIMIT);

    m->rate[0] = imu->gyro.x + m->bias[0];
    m->rate[1] = imu->gyro.y + m->bias[1];
    m->rate[2] = imu->gyro.z + m->bias[2];

    float gx = m->rate[0] + m->kp * (ex + hex);
    float gy = m->rate[1] + m->kp * (ey + hey);
    float gz = m->rate[2] + m->kp * (ez + hez);
    
    float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    q0 += dq0 * dt;
    q1 += dq1 * dt;
    q2 += dq2 * dt;
    q3 += dq3 * dt;

    float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if(norm < 1e-6f) return;
    norm = 1.0f / norm;

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

void Mahony_GetRate(const Mahony_t *m, float rate[3]){
    rate[0] = m->rate[0];
    rate[1] = m->rate[1];
    rate[2] = m->rate[2];
}

void Mahony_ResetHeading(Mahony_t *m){
    m->headingAligned = 0;
}