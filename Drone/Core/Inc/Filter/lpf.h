#ifndef INC_FILTER_LPF_H_
#define INC_FILTER_LPF_H_

#include <stdint.h>

typedef struct{
    float   cutoff_hz;
    float   y;
    uint8_t init;   /* seeds y on first sample, avoids startup transient */
} LPF_t;

void  LPF_Init(LPF_t *f, float cutoff_hz);
float LPF_Update(LPF_t *f, float x, float dt);

typedef struct{
    float b0, b1, b2, a1, a2;
    float x1, x2, y1, y2;
    uint8_t init;   /* seeds the state on first sample, avoids startup transient */
} Biquad_t;

void  Biquad_InitLowpass(Biquad_t *f, float cutoff_hz, float sample_hz);
float Biquad_Update(Biquad_t *f, float x);

#endif /* INC_FILTER_LPF_H_ */
