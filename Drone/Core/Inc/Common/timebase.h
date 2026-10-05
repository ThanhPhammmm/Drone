#ifndef INC_TIMEBASE_H_
#define INC_TIMEBASE_H_

#include <stdint.h>

uint32_t Time_Us(void);
uint32_t Time_UsAt(uint32_t cycles);
static inline int32_t Time_DiffUs(uint32_t a, uint32_t b){
	return (int32_t)(a - b);
}

#endif /* INC_TIMEBASE_H_ */
