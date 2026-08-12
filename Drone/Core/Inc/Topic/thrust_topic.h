#ifndef INC_TOPIC_THRUST_TOPIC_H_
#define INC_TOPIC_THRUST_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float thrust;
    uint8_t holdActive;
    uint32_t timestamp_us;
} Thrust_Data_t;

extern MutexTopic_t thrustTopic;

BaseType_t ThrustTopic_Init(void);
BaseType_t ThrustTopic_Publish(const Thrust_Data_t *thrust);
BaseType_t ThrustTopic_Copy(Thrust_Data_t *thrust, TickType_t timeout);

#endif /* INC_TOPIC_THRUST_TOPIC_H_ */
