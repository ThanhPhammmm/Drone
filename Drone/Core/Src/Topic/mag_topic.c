#include "mag_topic.h"

static Mag_Data_t magBuffer;
MutexTopic_t magTopic;

BaseType_t MagTopic_Init(void){
	return MutexTopic_Init(&magTopic, &magBuffer, sizeof(Mag_Data_t));
}

BaseType_t MagTopic_Publish(const Mag_Data_t *mag){
	return MutexTopic_Publish(&magTopic, mag);
}

BaseType_t MagTopic_Copy(Mag_Data_t *mag, TickType_t timeout){
	return MutexTopic_Copy(&magTopic, mag, timeout);
}
