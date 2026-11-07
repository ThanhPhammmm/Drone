#ifndef INC_TOPIC_GPS_TOPIC_H_
#define INC_TOPIC_GPS_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    uint8_t  fixType;       /* 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only */
    uint8_t  fixOk;         /* the receiver trusts this fix */
    uint8_t  numSV;         /* satellites used */
    int32_t  lat;           /* 1e-7 deg */
    int32_t  lon;           /* 1e-7 deg */
    float    altMsl;        /* m above mean sea level */
    float    hAcc;          /* m, horizontal accuracy estimate */
    float    vAcc;          /* m, vertical accuracy estimate */
    float    velN;          /* m/s, north */
    float    velE;          /* m/s, east */
    float    velD;          /* m/s, down */
    float    sAcc;          /* m/s, speed accuracy estimate */
    float    pDop;
    uint32_t iTOW;          /* ms, GPS time of week of the solution */
    uint32_t timestamp_us;  /* when the solution arrived (it describes the drone ~0.15 s earlier) */
} Gps_Data_t;

extern MutexTopic_t gpsTopic;

BaseType_t GpsTopic_Init(void);
BaseType_t GpsTopic_Publish(const Gps_Data_t *gps);
BaseType_t GpsTopic_Copy(Gps_Data_t *gps, TickType_t timeout);

#endif /* INC_TOPIC_GPS_TOPIC_H_ */
