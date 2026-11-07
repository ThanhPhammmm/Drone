#include "position_kalman_filter.h"

/* Process noise, as spectral densities: a variance per second of prediction */
#define POS_KF_ACCEL_NOISE          1.0f        /* (m/s^2)^2 / Hz, horizontal acceleration error the filter expects */
#define POS_KF_ACCEL_BIAS_NOISE     0.1f        /* (m/s^2)^2 / s, how fast the accel bias may wander */

/* Start-up uncertainty (1 sigma) */
#define POS_KF_INIT_V_STD           0.2f
#define POS_KF_INIT_ACCEL_BIAS_STD  0.3f

#define POS_KF_P_STD_MAX            10.0f       /* m: on the flow nothing measures p, so its doubt only grows; stop it here */
#define POS_KF_ACCEL_BIAS_LIMIT     1.5f        /* m/s^2, ~9 deg of attitude error */

static void PosKF_LimitAccelBias(PosKF_t *f){
    float *b = &f->kf.x[POS_KF_ACCEL_BIAS];
    if(*b >  POS_KF_ACCEL_BIAS_LIMIT) *b =  POS_KF_ACCEL_BIAS_LIMIT;
    if(*b < -POS_KF_ACCEL_BIAS_LIMIT) *b = -POS_KF_ACCEL_BIAS_LIMIT;
}

void PosKF_Init(PosKF_t *f, float p0){
    const float x0[POS_KF_N]   = { p0, 0.0f, 0.0f };
    const float var0[POS_KF_N] = {
        0.0f,       /* the start defines where 0 is */
        POS_KF_INIT_V_STD * POS_KF_INIT_V_STD,
        POS_KF_INIT_ACCEL_BIAS_STD * POS_KF_INIT_ACCEL_BIAS_STD
    };
    Kalman_Init(&f->kf, POS_KF_N, x0, var0);
}

/* p += v dt + (a - b) dt^2 / 2
 * v += (a - b) dt */
void PosKF_Predict(PosKF_t *f, float accel, float dt){
    float dt2 = dt * dt;
    float F[KALMAN_MAX_N][KALMAN_MAX_N] = {
        { 1.0f, dt,   -0.5f * dt2, 0.0f },
        { 0.0f, 1.0f, -dt,         0.0f },
        { 0.0f, 0.0f, 1.0f,        0.0f },
        { 0.0f, 0.0f, 0.0f,        1.0f },
    };
    float Q[KALMAN_MAX_N][KALMAN_MAX_N] = {{0.0f}};
    float qa = POS_KF_ACCEL_NOISE;
    Q[POS_KF_P][POS_KF_P] = qa * dt2 * dt / 3.0f;
    Q[POS_KF_P][POS_KF_V] = qa * dt2 / 2.0f;
    Q[POS_KF_V][POS_KF_P] = Q[POS_KF_P][POS_KF_V];
    Q[POS_KF_V][POS_KF_V] = qa * dt;
    Q[POS_KF_ACCEL_BIAS][POS_KF_ACCEL_BIAS] = POS_KF_ACCEL_BIAS_NOISE * dt;

    Kalman_Predict(&f->kf, F, Q);
    f->kf.x[POS_KF_P] += 0.5f * accel * dt2;
    f->kf.x[POS_KF_V] += accel * dt;
    Kalman_LimitVariance(&f->kf, POS_KF_P, POS_KF_P_STD_MAX * POS_KF_P_STD_MAX);
}

/* flow or GPS velocity = v */
uint8_t PosKF_UpdateVelocity(PosKF_t *f, float v, float std, float gate){
    const float H[KALMAN_MAX_N] = { 0.0f, 1.0f, 0.0f, 0.0f };
    uint8_t used = Kalman_Update(&f->kf, H, v, std * std, gate);
    PosKF_LimitAccelBias(f);
    return used;
}

/* GPS position = p */
uint8_t PosKF_UpdatePosition(PosKF_t *f, float p, float std, float gate){
    const float H[KALMAN_MAX_N] = { 1.0f, 0.0f, 0.0f, 0.0f };
    uint8_t used = Kalman_Update(&f->kf, H, p, std * std, gate);
    PosKF_LimitAccelBias(f);
    return used;
}

void PosKF_SetVelocity(PosKF_t *f, float v, float std){
    Kalman_SetState(&f->kf, POS_KF_V, v, std * std);
}
