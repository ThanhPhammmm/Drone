#ifndef INC_TOPIC_ALTITUDE_TOPIC_H_
#define INC_TOPIC_ALTITUDE_TOPIC_H_

#include "mutex_topic.h"
#include <stdint.h>

typedef struct{
    float altitude;         /* m */
    float verticalSpeed;    /* m/s */
    float accelUp;          /* m/s^2 */
    float accelBias;        /* m/s^2 */
    float liftVz;           /* m/s up, accelerometer only: how fast it leaves the ground during the take-off, else 0 */
    float baroBias;         /* m, baro error learned from the range sensor */
    float agl;              /* m, range sensor height above the floor below (tilt corrected) */

    uint8_t valid;
    uint8_t baroValid;
    uint8_t rangeValid;     /* the filter used the range sensor lately */
    uint8_t aglValid;       /* agl is a recent reading */
    uint8_t onFloor;        /* range sensor reads what it reads when landed */

    float dt;
    uint32_t timestamp_us;
} Altitude_Data_t;

extern MutexTopic_t altitudeTopic;

BaseType_t AltitudeTopic_Init(void);
BaseType_t AltitudeTopic_Publish(const Altitude_Data_t *alt);
BaseType_t AltitudeTopic_Copy(Altitude_Data_t *alt, TickType_t timeout);

#endif /* INC_TOPIC_ALTITUDE_TOPIC_H_ */
