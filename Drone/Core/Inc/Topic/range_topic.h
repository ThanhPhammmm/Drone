#ifndef INC_TOPIC_RANGE_TOPIC_H_
#define INC_TOPIC_RANGE_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float    distance_m;    /* m, along the sensor axis, not tilt corrected */
    uint8_t  rangeStatus;   /* VL53L1X_RangeStatus_t, 0 = valid */
    uint8_t  valid;
    uint32_t timestamp_us;
} Range_Data_t;

extern MutexTopic_t rangeTopic;

BaseType_t RangeTopic_Init(void);
BaseType_t RangeTopic_Publish(const Range_Data_t *range);
BaseType_t RangeTopic_Copy(Range_Data_t *range, TickType_t timeout);

#endif /* INC_TOPIC_RANGE_TOPIC_H_ */
