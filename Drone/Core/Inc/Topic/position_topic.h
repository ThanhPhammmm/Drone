#ifndef INC_TOPIC_POSITION_TOPIC_H_
#define INC_TOPIC_POSITION_TOPIC_H_

#include "topic.h"
#include <stdint.h>

/* what measured the velocity last */
#define POS_SOURCE_NONE         0
#define POS_SOURCE_FLOW         1
#define POS_SOURCE_GPS          2

/* Level frame: x = where the nose pointed at power-up (yaw 0), y = its right.
 * There is no compass, so yaw 0 is just the power-up heading. The GPS is
 * turned into this frame by gpsYawOffset, learned while flying on the flow. */
typedef struct{
    float x;                /* m, position: the flow added up (drifts slowly), the GPS when it is in use */
    float y;
    float vx;               /* m/s, level frame */
    float vy;
    float vFwd;             /* m/s, along the nose (level, heading frame) */
    float vRight;           /* m/s, to the right */
    float yaw;              /* rad, the heading used to turn between the two frames */

    uint8_t valid;          /* the flow (within 0.5 s) or the GPS (within 1 s) measured it, in the air */
    uint8_t source;         /* POS_SOURCE_* */

    uint8_t gpsAligned;     /* the level frame is tied to north: the GPS can be used */
    float   gpsYawOffset;   /* rad, from north to the level frame's x */
    float   gpsX;           /* m, the last GPS fix in the level frame (0 until aligned) */
    float   gpsY;

    uint32_t timestamp_us;
} Position_Data_t;

extern Topic_t positionTopic;

BaseType_t PositionTopic_Init(void);
BaseType_t PositionTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t PositionTopic_Publish(const Position_Data_t *pos);
BaseType_t PositionTopic_Copy(Position_Data_t *pos);

#endif /* INC_TOPIC_POSITION_TOPIC_H_ */
