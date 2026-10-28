#ifndef INC_SENSORDRIVER_PMW3901_PMW3901_DATA_TYPES_H_
#define INC_SENSORDRIVER_PMW3901_PMW3901_DATA_TYPES_H_

#include <stdint.h>

typedef enum{
    PMW3901_OK = 0,
    PMW3901_ERROR,
    PMW3901_INVALID_CHIP,
} PMW3901_Status_t;

typedef struct{
    uint8_t  motion;        /* MOTION register */
    uint8_t  observation;
    int16_t  deltaX;        /* counts moved since the last read */
    int16_t  deltaY;
    uint8_t  squal;         /* surface quality: how many features it sees */
    uint8_t  rawDataSum;
    uint8_t  maxRawData;
    uint8_t  minRawData;
    uint16_t shutter;       /* exposure: grows in the dark */
} PMW3901_Burst_t;

#endif /* INC_SENSORDRIVER_PMW3901_PMW3901_DATA_TYPES_H_ */
