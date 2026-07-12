#include "attitude_setpoint_topic.h"
#include "Const.h"

static AttitudeSetpoint_Data_t setpointBuffer[2];

Topic_t attitudeSetpointTopic;

BaseType_t AttitudeSetpointTopic_Init(void){
	return Topic_Init(&attitudeSetpointTopic, &setpointBuffer[0], &setpointBuffer[1], sizeof(AttitudeSetpoint_Data_t));
}

BaseType_t AttitudeSetpointTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&attitudeSetpointTopic, task, id);
}

BaseType_t AttitudeSetpointTopic_Publish(const AttitudeSetpoint_Data_t *setpoint){
	return Topic_Publish(&attitudeSetpointTopic, setpoint);
}

BaseType_t AttitudeSetpointTopic_Copy(AttitudeSetpoint_Data_t *setpoint){
	return Topic_Copy(&attitudeSetpointTopic, setpoint);
}
