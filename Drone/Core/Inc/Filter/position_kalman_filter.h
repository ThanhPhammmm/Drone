#ifndef INC_FILTER_POSITION_KALMAN_FILTER_H_
#define INC_FILTER_POSITION_KALMAN_FILTER_H_

#include "kalman.h"

/* Horizontal position Kalman filter, one per axis (x and y of the level frame), 3 states:
 *   p          m      position
 *   v          m/s    velocity
 *   accelBias  m/s^2  error of the horizontal acceleration: a small attitude
 *                     error lets some gravity leak into it */
enum{
    POS_KF_P = 0,
    POS_KF_V,
    POS_KF_ACCEL_BIAS,
    POS_KF_N
};

typedef struct{
    Kalman_t kf;
} PosKF_t;

void    PosKF_Init(PosKF_t *f, float p0);
void    PosKF_Predict(PosKF_t *f, float accel, float dt);
uint8_t PosKF_UpdateVelocity(PosKF_t *f, float v, float std, float gate);
void    PosKF_SetVelocity(PosKF_t *f, float v, float std);

static inline float PosKF_Get(const PosKF_t *f, uint8_t i){ return f->kf.x[i]; }

#endif /* INC_FILTER_POSITION_KALMAN_FILTER_H_ */
