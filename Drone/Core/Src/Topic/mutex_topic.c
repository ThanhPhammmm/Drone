#include "mutex_topic.h"
#include <string.h>

BaseType_t MutexTopic_Init(MutexTopic_t *topic, void *buffer, uint16_t size){
    topic->buffer = buffer;
    topic->size = size;
    topic->mutex = xSemaphoreCreateMutex();
    return (topic->mutex != NULL) ? pdPASS : pdFAIL;
}

BaseType_t MutexTopic_Publish(MutexTopic_t *topic, const void *data){
    if(xSemaphoreTake(topic->mutex, portMAX_DELAY) != pdTRUE) return pdFAIL;
    memcpy(topic->buffer, data, topic->size);
    xSemaphoreGive(topic->mutex);
    return pdPASS;
}

BaseType_t MutexTopic_Copy(MutexTopic_t *topic, void *data, TickType_t timeout){
    if(xSemaphoreTake(topic->mutex, timeout) != pdTRUE) return pdFAIL;
    memcpy(data, topic->buffer, topic->size);
    xSemaphoreGive(topic->mutex);
    return pdPASS;
}
