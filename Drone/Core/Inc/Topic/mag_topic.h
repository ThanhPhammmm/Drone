#ifndef INC_TOPIC_MAG_TOPIC_H_
#define INC_TOPIC_MAG_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float x, y, z;
    uint32_t timestamp_us;
} Mag_Data_t;

extern MutexTopic_t magTopic;

BaseType_t MagTopic_Init(void);
BaseType_t MagTopic_Publish(const Mag_Data_t *mag);
BaseType_t MagTopic_Copy(Mag_Data_t *mag, TickType_t timeout);

#endif /* INC_TOPIC_MAG_TOPIC_H_ */
