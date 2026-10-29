#include "kalman.h"
#include <math.h>
#include <string.h>

static void Kalman_Symmetrize(Kalman_t *kf){
    for(uint8_t i = 0; i < kf->n; i++){
        if(kf->P[i][i] < 0.0f) kf->P[i][i] = 0.0f;
        for(uint8_t j = i + 1; j < kf->n; j++){
            float m = 0.5f * (kf->P[i][j] + kf->P[j][i]);
            kf->P[i][j] = m;
            kf->P[j][i] = m;
        }
    }
}

void Kalman_Init(Kalman_t *kf, uint8_t n, const float *x0, const float *var0){
    memset(kf, 0, sizeof(*kf));
    kf->n = (n > KALMAN_MAX_N) ? KALMAN_MAX_N : n;
    for(uint8_t i = 0; i < kf->n; i++){
        kf->x[i]    = (x0 != NULL) ? x0[i] : 0.0f;
        kf->P[i][i] = var0[i];
    }
}

void Kalman_Predict(Kalman_t *kf, const float F[KALMAN_MAX_N][KALMAN_MAX_N],
                    const float Q[KALMAN_MAX_N][KALMAN_MAX_N]){
    uint8_t n = kf->n;
    float x[KALMAN_MAX_N];
    float FP[KALMAN_MAX_N][KALMAN_MAX_N];

    for(uint8_t i = 0; i < n; i++){
        x[i] = 0.0f;
        for(uint8_t k = 0; k < n; k++) x[i] += F[i][k] * kf->x[k];
    }
    for(uint8_t i = 0; i < n; i++){
        for(uint8_t j = 0; j < n; j++){
            FP[i][j] = 0.0f;
            for(uint8_t k = 0; k < n; k++) FP[i][j] += F[i][k] * kf->P[k][j];
        }
    }
    for(uint8_t i = 0; i < n; i++){
        kf->x[i] = x[i];
        for(uint8_t j = 0; j < n; j++){
            float s = Q[i][j];
            for(uint8_t k = 0; k < n; k++) s += FP[i][k] * F[j][k];
            kf->P[i][j] = s;
        }
    }
    Kalman_Symmetrize(kf);
}

uint8_t Kalman_Update(Kalman_t *kf, const float H[KALMAN_MAX_N], float z, float R, float gate){
    uint8_t n = kf->n;
    float PHt[KALMAN_MAX_N];

    float S = R;
    float y = z;
    for(uint8_t i = 0; i < n; i++){
        PHt[i] = 0.0f;
        for(uint8_t k = 0; k < n; k++) PHt[i] += kf->P[i][k] * H[k];
        S += H[i] * PHt[i];
        y -= H[i] * kf->x[i];
    }
    if(!(S > 0.0f)) return 0;
    if(gate > 0.0f && y * y > gate * S) return 0;

    for(uint8_t i = 0; i < n; i++){
        float K = PHt[i] / S;
        kf->x[i] += K * y;
        for(uint8_t j = 0; j < n; j++) kf->P[i][j] -= K * PHt[j];  /* P -= K (H P), H P = PHt' */
    }
    Kalman_Symmetrize(kf);
    return 1;
}

void Kalman_SetState(Kalman_t *kf, uint8_t i, float value, float var){
    if(i >= kf->n) return;
    kf->x[i] = value;
    for(uint8_t j = 0; j < kf->n; j++){
        kf->P[i][j] = 0.0f;
        kf->P[j][i] = 0.0f;
    }
    kf->P[i][i] = var;
}

void Kalman_LimitVariance(Kalman_t *kf, uint8_t i, float maxVar){
    if(i >= kf->n || kf->P[i][i] <= maxVar) return;
    /* scale row and column i: P stays a valid covariance */
    float s = sqrtf(maxVar / kf->P[i][i]);
    for(uint8_t j = 0; j < kf->n; j++){
        kf->P[i][j] *= s;
        kf->P[j][i] *= s;
    }
    kf->P[i][i] = maxVar;
}
