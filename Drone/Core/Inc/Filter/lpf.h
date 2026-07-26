#ifndef INC_FILTER_LPF_H_
#define INC_FILTER_LPF_H_

#include <stdint.h>

typedef struct{
    float   cutoff_hz;
    float   y;      /* filtered output */
    uint8_t init;   /* seeds y on first sample, avoids startup transient */
} LPF_t;

void  LPF_Init(LPF_t *f, float cutoff_hz);
float LPF_Update(LPF_t *f, float x, float dt);

#endif /* INC_FILTER_LPF_H_ */
