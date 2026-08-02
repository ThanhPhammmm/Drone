#ifndef INC_TOPIC_ALTITUDE_TOPIC_H_
#define INC_TOPIC_ALTITUDE_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float altitude;         /* m */
    float verticalSpeed;    /* m/s */
    float accelUp;          /* m/s^2 */
    float accelBias;        /* m/s^2 */

    uint8_t valid;
    uint8_t baroValid;

    float dt;
    uint32_t timestamp_us;
} Altitude_Data_t;

extern MutexTopic_t altitudeTopic;

BaseType_t AltitudeTopic_Init(void);
BaseType_t AltitudeTopic_Publish(const Altitude_Data_t *alt);
BaseType_t AltitudeTopic_Copy(Altitude_Data_t *alt, TickType_t timeout);

#endif /* INC_TOPIC_ALTITUDE_TOPIC_H_ */
