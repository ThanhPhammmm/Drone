#ifndef INC_FILTER_KALMAN_H_
#define INC_FILTER_KALMAN_H_

#include <stdint.h>

/* Small linear Kalman filter, up to KALMAN_MAX_N states.
 * Measurements are scalar and applied one at a time, so there is no matrix
 * inversion: S = H P H' + R is a number. */

#define KALMAN_MAX_N    4

typedef struct{
    uint8_t n;
    float x[KALMAN_MAX_N];                  /* state */
    float P[KALMAN_MAX_N][KALMAN_MAX_N];    /* covariance */
} Kalman_t;

void Kalman_Init(Kalman_t *kf, uint8_t n, const float *x0, const float *var0);

/* x = F x,  P = F P F' + Q */
void Kalman_Predict(Kalman_t *kf, const float F[KALMAN_MAX_N][KALMAN_MAX_N],
                    const float Q[KALMAN_MAX_N][KALMAN_MAX_N]);

/* z = H x + noise of variance R.
 * gate: largest accepted innovation^2 / S (9 = 3 sigma), 0 = accept all.
 * 1 = used, 0 = rejected by the gate */
uint8_t Kalman_Update(Kalman_t *kf, const float H[KALMAN_MAX_N], float z, float R, float gate);

/* Force state i to a known value with variance var, and forget how it was
 * tied to the other states. */
void Kalman_SetState(Kalman_t *kf, uint8_t i, float value, float var);

/* Keep the variance of state i at or below maxVar (a state no sensor sees
 * grows without end). */
void Kalman_LimitVariance(Kalman_t *kf, uint8_t i, float maxVar);

#endif /* INC_FILTER_KALMAN_H_ */
