#ifndef INC_SENSORDRIVER_NEO_M8N_NEO_M8N_DATA_TYPES_H_
#define INC_SENSORDRIVER_NEO_M8N_NEO_M8N_DATA_TYPES_H_

#include <stdint.h>

typedef enum{
    NEO_M8N_OK = 0,
    NEO_M8N_ERROR
} NEO_M8N_Status_t;

/* UBX NAV-PVT: the fields the drone uses */
typedef struct{
    uint32_t iTOW;          /* ms, GPS time of week of this solution */
    uint8_t  fixType;       /* UBX_FIX_*: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only */
    uint8_t  fixOk;         /* gnssFixOK: the receiver trusts this fix */
    uint8_t  numSV;         /* satellites used */
    int32_t  lon;           /* 1e-7 deg */
    int32_t  lat;           /* 1e-7 deg */
    int32_t  hMSL;          /* mm above mean sea level */
    uint32_t hAcc;          /* mm, horizontal accuracy estimate */
    uint32_t vAcc;          /* mm, vertical accuracy estimate */
    int32_t  velN;          /* mm/s */
    int32_t  velE;
    int32_t  velD;
    uint32_t sAcc;          /* mm/s, speed accuracy estimate */
    uint16_t pDOP;          /* 0.01 */
} NEO_M8N_Pvt_t;

#endif /* INC_SENSORDRIVER_NEO_M8N_NEO_M8N_DATA_TYPES_H_ */
