#ifndef INC_TOPIC_RATE_SETPOINT_TOPIC_H_
#define INC_TOPIC_RATE_SETPOINT_TOPIC_H_

#include "topic.h"
#include <stdint.h>

typedef struct{
    float rollRate;
    float pitchRate;
    float yawRate;

    uint32_t timestamp_us;

} RateSetpoint_Data_t;

extern Topic_t rateSetpointTopic;

BaseType_t RateSetpointTopic_Init(void);
BaseType_t RateSetpointTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t RateSetpointTopic_Publish(const RateSetpoint_Data_t *rate);
BaseType_t RateSetpointTopic_Copy(RateSetpoint_Data_t *rate);

#endif /* INC_TOPIC_RATE_SETPOINT_TOPIC_H_ */
