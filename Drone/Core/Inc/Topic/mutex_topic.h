#ifndef INC_TOPIC_MUTEX_TOPIC_H_
#define INC_TOPIC_MUTEX_TOPIC_H_

#include "FreeRTOS.h"
#include "semphr.h"
#include <stdint.h>

typedef struct{
    void              *buffer;
    uint16_t           size;
    SemaphoreHandle_t  mutex;
} MutexTopic_t;

BaseType_t MutexTopic_Init(MutexTopic_t *topic, void *buffer, uint16_t size);
BaseType_t MutexTopic_Publish(MutexTopic_t *topic, const void *data);
BaseType_t MutexTopic_Copy(MutexTopic_t *topic, void *data, TickType_t timeout);

#endif /* INC_TOPIC_MUTEX_TOPIC_H_ */
