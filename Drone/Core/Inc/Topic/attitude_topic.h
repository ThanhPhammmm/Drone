#ifndef INC_TOPIC_ATTITUDE_TOPIC_H_
#define INC_TOPIC_ATTITUDE_TOPIC_H_

#include "topic.h"
#include <stdint.h>

typedef struct{
    float q0;
    float q1;
    float q2;
    float q3;

    float q[4];

    float roll;
    float pitch;
    float yaw;

    /* bias-corrected measured body rates (rad/s), fed to the rate controller */
    float rollRate;
    float pitchRate;
    float yawRate;

    float biasX;
    float biasY;
    float biasZ;

    float dt;

    uint32_t timestamp_us;

} Attitude_Data_t;

extern Topic_t attitudeTopic;

BaseType_t AttitudeTopic_Init(void);
BaseType_t AttitudeTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t AttitudeTopic_Publish(const Attitude_Data_t *att);
BaseType_t AttitudeTopic_Copy(Attitude_Data_t *att);

#endif /* INC_TOPIC_ATTITUDE_TOPIC_H_ */
