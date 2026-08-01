#ifndef INC_SENSORDRIVER_BMP388_BMP388_H_
#define INC_SENSORDRIVER_BMP388_BMP388_H_

#include "bmp388_data_types.h"

typedef struct{
    BMP388_Raw_t       raw;
    BMP388_Calib_t     calib;
    BMP388_GroundRef_t groundRef;

    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;
    uint8_t  addr;
} BMP388_Handle_t;

extern BMP388_Handle_t bmp388;

BMP388_Status_t BMP388_Init(void);
uint8_t BMP388_DataReady(void);
BMP388_Status_t BMP388_Read(void);
void BMP388_GroundRefReset(void);
void BMP388_GroundRefAccumulate(void);

#endif /* INC_SENSORDRIVER_BMP388_BMP388_H_ */
