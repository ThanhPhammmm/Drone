#include "alttitude_kalman_filter.h"

#define ALT_KF_ACCEL_NOISE          1.0f        /* (m/s^2)^2 / Hz, accelUp error the filter expects */
#define ALT_KF_ACCEL_BIAS_NOISE     3.0f        /* (m/s^2)^2 / s, how fast the accel bias may wander */
#define ALT_KF_BARO_BIAS_NOISE      0.01f       /* m^2 / s, how fast the baro error may wander */

/* Measurement noise (1 sigma) */
#define ALT_KF_BARO_STD             0.20f       /* m */
#define ALT_KF_RANGE_GATE           9.0f        /* innovation^2 / S: 3 sigma */

/* Start-up uncertainty (1 sigma) */
#define ALT_KF_INIT_Z_STD           0.5f
#define ALT_KF_INIT_VZ_STD          0.2f
#define ALT_KF_INIT_ACCEL_BIAS_STD  0.5f

/* Ground: vz and accel bias are known (lift-off speed and standing reading) */
#define ALT_KF_GROUND_VZ_STD        0.05f
#define ALT_KF_GROUND_BIAS_STD      0.05f
#define ALT_KF_GROUND_Z_STD         0.10f       /* z keeps this much doubt, so the baro still corrects it */

#define ALT_KF_ACCEL_BIAS_LIMIT     2.0f        /* m/s^2 */

static void AltKF_LimitAccelBias(AltKF_t *f){
    float *b = &f->kf.x[ALT_KF_ACCEL_BIAS];
    if(*b >  ALT_KF_ACCEL_BIAS_LIMIT) *b =  ALT_KF_ACCEL_BIAS_LIMIT;
    if(*b < -ALT_KF_ACCEL_BIAS_LIMIT) *b = -ALT_KF_ACCEL_BIAS_LIMIT;
}

void AltKF_Init(AltKF_t *f, float z0){
    const float x0[ALT_KF_N]   = { z0, 0.0f, 0.0f, 0.0f };
    const float var0[ALT_KF_N] = {
        ALT_KF_INIT_Z_STD * ALT_KF_INIT_Z_STD,
        ALT_KF_INIT_VZ_STD * ALT_KF_INIT_VZ_STD,
        ALT_KF_INIT_ACCEL_BIAS_STD * ALT_KF_INIT_ACCEL_BIAS_STD,
        0.0f    /* baro is the height reference until the range sensor says otherwise */
    };
    Kalman_Init(&f->kf, ALT_KF_N, x0, var0);
}

/* z  += vz dt + (a - b) dt^2 / 2
 * vz += (a - b) dt */
void AltKF_Predict(AltKF_t *f, float accelUp, float dt, uint8_t learnBaroBias){
    float dt2 = dt * dt;
    float F[KALMAN_MAX_N][KALMAN_MAX_N] = {
        { 1.0f, dt,   -0.5f * dt2, 0.0f },
        { 0.0f, 1.0f, -dt,         0.0f },
        { 0.0f, 0.0f, 1.0f,        0.0f },
        { 0.0f, 0.0f, 0.0f,        1.0f },
    };
    float Q[KALMAN_MAX_N][KALMAN_MAX_N] = {{0.0f}};
    float qa = ALT_KF_ACCEL_NOISE;
    Q[ALT_KF_Z][ALT_KF_Z]   = qa * dt2 * dt / 3.0f;
    Q[ALT_KF_Z][ALT_KF_VZ]  = qa * dt2 / 2.0f;
    Q[ALT_KF_VZ][ALT_KF_Z]  = Q[ALT_KF_Z][ALT_KF_VZ];
    Q[ALT_KF_VZ][ALT_KF_VZ] = qa * dt;
    Q[ALT_KF_ACCEL_BIAS][ALT_KF_ACCEL_BIAS] = ALT_KF_ACCEL_BIAS_NOISE * dt;
    if(learnBaroBias) Q[ALT_KF_BARO_BIAS][ALT_KF_BARO_BIAS] = ALT_KF_BARO_BIAS_NOISE * dt;

    Kalman_Predict(&f->kf, F, Q);
    f->kf.x[ALT_KF_Z]  += 0.5f * accelUp * dt2;
    f->kf.x[ALT_KF_VZ] += accelUp * dt;
}

/* baro = z + baroBias */
void AltKF_UpdateBaro(AltKF_t *f, float baroAltitude){
    const float H[KALMAN_MAX_N] = { 1.0f, 0.0f, 0.0f, 1.0f };
    Kalman_Update(&f->kf, H, baroAltitude, ALT_KF_BARO_STD * ALT_KF_BARO_STD, 0.0f);
    AltKF_LimitAccelBias(f);
}

/* range height (tilt corrected, shifted to the z of the floor) = z */
uint8_t AltKF_UpdateRange(AltKF_t *f, float height, float std){
    const float H[KALMAN_MAX_N] = { 1.0f, 0.0f, 0.0f, 0.0f };
    uint8_t used = Kalman_Update(&f->kf, H, height, std * std, ALT_KF_RANGE_GATE);
    AltKF_LimitAccelBias(f);
    return used;
}

void AltKF_SetGround(AltKF_t *f, float vz, float accelBias){
    Kalman_SetState(&f->kf, ALT_KF_VZ, vz, ALT_KF_GROUND_VZ_STD * ALT_KF_GROUND_VZ_STD);
    Kalman_SetState(&f->kf, ALT_KF_ACCEL_BIAS, accelBias, ALT_KF_GROUND_BIAS_STD * ALT_KF_GROUND_BIAS_STD);
    /* vz is pinned, so without this z would only ever follow it */
    if(f->kf.P[ALT_KF_Z][ALT_KF_Z] < ALT_KF_GROUND_Z_STD * ALT_KF_GROUND_Z_STD){
        f->kf.P[ALT_KF_Z][ALT_KF_Z] = ALT_KF_GROUND_Z_STD * ALT_KF_GROUND_Z_STD;
    }
    AltKF_LimitAccelBias(f);
}
