#ifndef INC_TOPIC_TOPIC_H_
#define INC_TOPIC_TOPIC_H_

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define TOPIC_MAX_SUBSCRIBERS 10

typedef struct{
    void             *buffer[2];
    volatile uint8_t  write_index;
    volatile uint32_t seq;
    uint16_t          size;
    TaskHandle_t      subscribers[TOPIC_MAX_SUBSCRIBERS];
} Topic_t;

BaseType_t Topic_Init(Topic_t *topic, void *buf0, void *buf1, uint16_t size);
BaseType_t Topic_Subscribe(Topic_t *topic, TaskHandle_t task, const uint8_t id);
BaseType_t Topic_Publish(Topic_t *topic, const void *data);
BaseType_t Topic_Copy(Topic_t *topic, void *data);

#endif /* INC_TOPIC_TOPIC_H_ */
