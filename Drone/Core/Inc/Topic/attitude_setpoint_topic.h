#ifndef INC_TOPIC_ATTITUDE_SETPOINT_TOPIC_H_
#define INC_TOPIC_ATTITUDE_SETPOINT_TOPIC_H_

#include "topic.h"
#include <stdint.h>

typedef struct{
    /* target roll/pitch angle (rad); yaw has no heading reference (no
     * magnetometer) so it is commanded directly as a rate instead */
    float roll;
    float pitch;
    float yawRate;

    uint32_t timestamp_us;

} AttitudeSetpoint_Data_t;

extern Topic_t attitudeSetpointTopic;

BaseType_t AttitudeSetpointTopic_Init(void);
BaseType_t AttitudeSetpointTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t AttitudeSetpointTopic_Publish(const AttitudeSetpoint_Data_t *setpoint);
BaseType_t AttitudeSetpointTopic_Copy(AttitudeSetpoint_Data_t *setpoint);

#endif /* INC_TOPIC_ATTITUDE_SETPOINT_TOPIC_H_ */
