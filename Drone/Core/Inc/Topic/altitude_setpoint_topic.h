#ifndef INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_
#define INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float climbRate;

    /* Raw throttle 0..1, used directly whenever hold is disengaged. */
    float manualThrust;

    /* 1 = closed-loop altitude hold, 0 = manual thrust passthrough. */
    uint8_t holdEnabled;

    uint32_t timestamp_us;

} AltitudeSetpoint_Data_t;

extern MutexTopic_t altitudeSetpointTopic;

BaseType_t AltitudeSetpointTopic_Init(void);
BaseType_t AltitudeSetpointTopic_Publish(const AltitudeSetpoint_Data_t *setpoint);
BaseType_t AltitudeSetpointTopic_Copy(AltitudeSetpoint_Data_t *setpoint, TickType_t timeout);

#endif /* INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_ */
