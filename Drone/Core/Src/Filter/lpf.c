#include "lpf.h"
#include <math.h>

#define LPF_TWO_PI 6.28318530718f
#define LPF_PI     3.14159265359f

void LPF_Init(LPF_t *f, float cutoff_hz){
    f->cutoff_hz = cutoff_hz;
    f->y = 0.0f;
    f->init = 0;
}

float LPF_Update(LPF_t *f, float x, float dt){
    if(!f->init){          /* seed with first real sample, no ramp-in from 0 */
        f->y = x;
        f->init = 1;
        return f->y;
    }
    if(dt <= 0.0f) return f->y;   /* guards the dt=0 first-sample case elsewhere */

    float rc	= 1.0f / (LPF_TWO_PI * f->cutoff_hz);
    float alpha = dt / (rc + dt);
    f->y += alpha * (x - f->y);
    return f->y;
}

void Biquad_InitLowpass(Biquad_t *f, float cutoff_hz, float sample_hz){
    float k    = tanf(LPF_PI * cutoff_hz / sample_hz);
    float q    = 0.70710678f;
    float norm = 1.0f / (1.0f + k / q + k * k);

    f->b0 = k * k * norm;
    f->b1 = 2.0f * f->b0;
    f->b2 = f->b0;
    f->a1 = 2.0f * (k * k - 1.0f) * norm;
    f->a2 = (1.0f - k / q + k * k) * norm;

    f->x1 = f->x2 = f->y1 = f->y2 = 0.0f;
    f->init = 0;
}

float Biquad_Update(Biquad_t *f, float x){
    if(!f->init){          /* seed with first real sample, no ramp-in from 0 */
        f->x1 = f->x2 = f->y1 = f->y2 = x;
        f->init = 1;
        return x;
    }
    float y = f->b0 * x + f->b1 * f->x1 + f->b2 * f->x2 - f->a1 * f->y1 - f->a2 * f->y2;
    f->x2 = f->x1;
    f->x1 = x;
    f->y2 = f->y1;
    f->y1 = y;
    return y;
}
