#ifndef INC_TOPIC_RC_TOPIC_H_
#define INC_TOPIC_RC_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float roll;             /* -1..1 */
    float pitch;            /* -1..1 */
    float yaw;              /* -1..1 */
    float throttle;         /*  0..1 */

    uint8_t mode;           /* RC_Mode_t actually in force */
    uint8_t linkOk;

    uint32_t timestamp_us;
} RC_Data_t;

extern MutexTopic_t rcTopic;

BaseType_t RCTopic_Init(void);
BaseType_t RCTopic_Publish(const RC_Data_t *rc);
BaseType_t RCTopic_Copy(RC_Data_t *rc, TickType_t timeout);

#endif /* INC_TOPIC_RC_TOPIC_H_ */
