#ifndef INC_TOPIC_TILT_TOPIC_H_
#define INC_TOPIC_TILT_TOPIC_H_

#include "topic.h"
#include <stdint.h>

/* Roll/pitch the position controller asks for (POS_HOLD). The attitude
 * controller flies these instead of the stick angles while active is set. */
typedef struct{
    float roll;             /* rad, + = right side down */
    float pitch;            /* rad, + = nose up */
    uint8_t active;
    uint32_t timestamp_us;
} Tilt_Data_t;

extern Topic_t tiltTopic;

BaseType_t TiltTopic_Init(void);
BaseType_t TiltTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t TiltTopic_Publish(const Tilt_Data_t *tilt);
BaseType_t TiltTopic_Copy(Tilt_Data_t *tilt);

#endif /* INC_TOPIC_TILT_TOPIC_H_ */
