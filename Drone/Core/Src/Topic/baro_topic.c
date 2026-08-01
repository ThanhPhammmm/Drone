#include "baro_topic.h"

static Baro_Data_t baroBuffer;
MutexTopic_t baroTopic;

BaseType_t BaroTopic_Init(void){
	return MutexTopic_Init(&baroTopic, &baroBuffer, sizeof(Baro_Data_t));
}

BaseType_t BaroTopic_Publish(const Baro_Data_t *baro){
	return MutexTopic_Publish(&baroTopic, baro);
}

BaseType_t BaroTopic_Copy(Baro_Data_t *baro, TickType_t timeout){
	return MutexTopic_Copy(&baroTopic, baro, timeout);
}