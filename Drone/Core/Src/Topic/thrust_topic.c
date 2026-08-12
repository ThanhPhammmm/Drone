#include "thrust_topic.h"

static Thrust_Data_t thrustBuffer;
MutexTopic_t thrustTopic;

BaseType_t ThrustTopic_Init(void){
	return MutexTopic_Init(&thrustTopic, &thrustBuffer, sizeof(Thrust_Data_t));
}

BaseType_t ThrustTopic_Publish(const Thrust_Data_t *thrust){
	return MutexTopic_Publish(&thrustTopic, thrust);
}

BaseType_t ThrustTopic_Copy(Thrust_Data_t *thrust, TickType_t timeout){
	return MutexTopic_Copy(&thrustTopic, thrust, timeout);
}
