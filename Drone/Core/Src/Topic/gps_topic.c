#include "gps_topic.h"

static Gps_Data_t gpsBuffer;
MutexTopic_t gpsTopic;

BaseType_t GpsTopic_Init(void){
	return MutexTopic_Init(&gpsTopic, &gpsBuffer, sizeof(Gps_Data_t));
}

BaseType_t GpsTopic_Publish(const Gps_Data_t *gps){
	return MutexTopic_Publish(&gpsTopic, gps);
}

BaseType_t GpsTopic_Copy(Gps_Data_t *gps, TickType_t timeout){
	return MutexTopic_Copy(&gpsTopic, gps, timeout);
}