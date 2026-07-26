#include "lpf.h"

#define LPF_TWO_PI 6.28318530718f

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
