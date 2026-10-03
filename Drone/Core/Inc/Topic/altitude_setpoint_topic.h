#ifndef INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_
#define INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float climbRate;    // m/s, ALT_HOLD only
    float manualThrust; // 0..1, ANGLE collective (and ALT_HOLD fallback)
    uint8_t mode;       // FlightMode_t in force
    uint32_t timestamp_us;
} AltitudeSetpoint_Data_t;

extern MutexTopic_t altitudeSetpointTopic;

BaseType_t AltitudeSetpointTopic_Init(void);
BaseType_t AltitudeSetpointTopic_Publish(const AltitudeSetpoint_Data_t *setpoint);
BaseType_t AltitudeSetpointTopic_Copy(AltitudeSetpoint_Data_t *setpoint, TickType_t timeout);

#endif /* INC_TOPIC_ALTITUDE_SETPOINT_TOPIC_H_ */
