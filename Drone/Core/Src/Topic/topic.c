#include "topic.h"
#include "stm32f4xx.h"
#include <string.h>

BaseType_t Topic_Init(Topic_t *topic, void *buf0, void *buf1, uint16_t size){
    topic->buffer[0] = buf0;
    topic->buffer[1] = buf1;
    topic->write_index = 0;
    topic->seq = 0;
    topic->size = size;
    memset(topic->subscribers, 0, sizeof(topic->subscribers));
    return pdPASS;
}

BaseType_t Topic_Subscribe(Topic_t *topic, TaskHandle_t task, const uint8_t id){
    if(id >= TOPIC_MAX_SUBSCRIBERS) return pdFAIL;
    topic->subscribers[id] = task;
    return pdPASS;
}

BaseType_t Topic_Publish(Topic_t *topic, const void *data){
    uint8_t spare = topic->write_index ^ 1u;
    memcpy(topic->buffer[spare], data, topic->size);
    __DMB();
    topic->write_index = spare;
    topic->seq++;

    for(uint8_t id = 0; id < TOPIC_MAX_SUBSCRIBERS; id++){
        if(topic->subscribers[id] != NULL){
            xTaskNotifyGive(topic->subscribers[id]);
        }
    }
    return pdPASS;
}

BaseType_t Topic_Copy(Topic_t *topic, void *data){
    uint32_t s;
    uint8_t  idx;
    do {
        s   = topic->seq;
        idx = topic->write_index;
        memcpy(data, topic->buffer[idx], topic->size);
        __DMB();
    } while(s != topic->seq);
    return pdPASS;
}
