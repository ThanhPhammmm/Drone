#include "altitude_topic.h"

static Altitude_Data_t altitudeBuffer;
MutexTopic_t altitudeTopic;

BaseType_t AltitudeTopic_Init(void){
	return MutexTopic_Init(&altitudeTopic, &altitudeBuffer, sizeof(Altitude_Data_t));
}

BaseType_t AltitudeTopic_Publish(const Altitude_Data_t *alt){
	return MutexTopic_Publish(&altitudeTopic, alt);
}

BaseType_t AltitudeTopic_Copy(Altitude_Data_t *alt, TickType_t timeout){
	return MutexTopic_Copy(&altitudeTopic, alt, timeout);
}
