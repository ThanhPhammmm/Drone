#ifndef INC_TOPIC_BARO_TOPIC_H_
#define INC_TOPIC_BARO_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float    pressure_pa;
    float    temperature_c;
    float    altitude_m;
    uint32_t timestamp_us;
} Baro_Data_t;

extern MutexTopic_t baroTopic;

BaseType_t BaroTopic_Init(void);
BaseType_t BaroTopic_Publish(const Baro_Data_t *baro);
BaseType_t BaroTopic_Copy(Baro_Data_t *baro, TickType_t timeout);

#endif /* INC_TOPIC_BARO_TOPIC_H_ */