#ifndef INC_TOPIC_POSITION_TOPIC_H_
#define INC_TOPIC_POSITION_TOPIC_H_

#include "topic.h"
#include <stdint.h>

/* Level frame: x = where the nose pointed at power-up (yaw 0), y = its right.
 * There is no compass indoors, so yaw 0 is just the power-up heading. */
typedef struct{
    float x;                /* m, position, from the flow added up: drifts slowly */
    float y;
    float vx;               /* m/s, level frame */
    float vy;
    float vFwd;             /* m/s, along the nose (level, heading frame) */
    float vRight;           /* m/s, to the right */
    float yaw;              /* rad, the heading used to turn between the two frames */

    uint8_t valid;          /* flow measured the velocity within the last 0.5 s, in the air */

    uint32_t timestamp_us;
} Position_Data_t;

extern Topic_t positionTopic;

BaseType_t PositionTopic_Init(void);
BaseType_t PositionTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t PositionTopic_Publish(const Position_Data_t *pos);
BaseType_t PositionTopic_Copy(Position_Data_t *pos);

#endif /* INC_TOPIC_POSITION_TOPIC_H_ */
