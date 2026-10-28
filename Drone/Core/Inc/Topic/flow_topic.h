#ifndef INC_TOPIC_FLOW_TOPIC_H_
#define INC_TOPIC_FLOW_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    int16_t  deltaX;        /* counts moved since the previous sample */
    int16_t  deltaY;
    uint32_t dt_us;         /* time those counts were gathered over */
    uint8_t  squal;         /* surface quality */
    uint8_t  valid;         /* squal good enough to trust */
    uint32_t timestamp_us;
} Flow_Data_t;

extern MutexTopic_t flowTopic;

BaseType_t FlowTopic_Init(void);
BaseType_t FlowTopic_Publish(const Flow_Data_t *flow);
BaseType_t FlowTopic_Copy(Flow_Data_t *flow, TickType_t timeout);

#endif /* INC_TOPIC_FLOW_TOPIC_H_ */
