#ifndef INC_SENSORDRIVER_BMP388_BMP388_DATA_TYPES_H_
#define INC_SENSORDRIVER_BMP388_BMP388_DATA_TYPES_H_

#include <stdint.h>

typedef struct{
    uint32_t pressure;
    uint32_t temperature;
} BMP388_Raw_t;

typedef struct{
    float t1, t2, t3;
    float p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11;
    float t_lin;
} BMP388_Calib_t;

typedef struct{
    float    pressureSum;
    uint16_t samples;
    float    pressure_pa;
    uint8_t  valid;
} BMP388_GroundRef_t;

typedef enum{
    BMP388_OK = 0,
    BMP388_ERROR,
    BMP388_TIMEOUT,
    BMP388_INVALID_CHIP,
    BMP388_CONFIG_ERROR,
} BMP388_Status_t;

#endif /* INC_SENSORDRIVER_BMP388_BMP388_DATA_TYPES_H_ */