#include "altitude_setpoint_topic.h"

static AltitudeSetpoint_Data_t altitudeSetpointBuffer;
MutexTopic_t altitudeSetpointTopic;

BaseType_t AltitudeSetpointTopic_Init(void){
	return MutexTopic_Init(&altitudeSetpointTopic, &altitudeSetpointBuffer, sizeof(AltitudeSetpoint_Data_t));
}

BaseType_t AltitudeSetpointTopic_Publish(const AltitudeSetpoint_Data_t *setpoint){
	return MutexTopic_Publish(&altitudeSetpointTopic, setpoint);
}

BaseType_t AltitudeSetpointTopic_Copy(AltitudeSetpoint_Data_t *setpoint, TickType_t timeout){
	return MutexTopic_Copy(&altitudeSetpointTopic, setpoint, timeout);
}
