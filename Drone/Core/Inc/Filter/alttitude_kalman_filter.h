#ifndef INC_FILTER_ALTTITUDE_KALMAN_FILTER_H_
#define INC_FILTER_ALTTITUDE_KALMAN_FILTER_H_

#include "kalman.h"

/* Height Kalman filter, 4 states:
 *   z          m      height (up), 0 near where the baro took its ground reference
 *   vz         m/s    vertical speed (up)
 *   accelBias  m/s^2  error of accelUp
 *   baroBias   m      baro error (drift, prop wash); only learned while the
 *                     range sensor gives a second height, else it is held */
enum{
    ALT_KF_Z = 0,
    ALT_KF_VZ,
    ALT_KF_ACCEL_BIAS,
    ALT_KF_BARO_BIAS,
    ALT_KF_N
};

typedef struct{
    Kalman_t kf;
} AltKF_t;

void    AltKF_Init(AltKF_t *f, float z0);
void    AltKF_Predict(AltKF_t *f, float accelUp, float dt, uint8_t learnBaroBias);
void    AltKF_UpdateBaro(AltKF_t *f, float baroAltitude);
uint8_t AltKF_UpdateRange(AltKF_t *f, float height, float std);   /* 1 = used, 0 = outlier */
void    AltKF_SetGround(AltKF_t *f, float vz, float accelBias);    /* on the ground: vz and bias are known */
static inline float AltKF_Get(const AltKF_t *f, uint8_t i){ return f->kf.x[i]; }

#endif /* INC_FILTER_ALTTITUDE_KALMAN_FILTER_H_ */
