#ifndef INC_SENSORDRIVER_QMC5883_QMC5883_DATA_TYPES_H_
#define INC_SENSORDRIVER_QMC5883_QMC5883_DATA_TYPES_H_

#include <stdint.h>

typedef struct{
    int16_t x, y, z;
} QMC5883_Raw_t;

typedef struct{
    float x, y, z;
} QMC5883_Field_t;

typedef struct{
    float   offset[3];
    uint8_t calibrated;
} QMC5883_Calib_t;

typedef enum{
    QMC5883_OK = 0,
    QMC5883_ERROR,
    QMC5883_TIMEOUT,
    QMC5883_INVALID_CHIP,
} QMC5883_Status_t;

#endif /* INC_SENSORDRIVER_QMC5883_QMC5883_DATA_TYPES_H_ */
