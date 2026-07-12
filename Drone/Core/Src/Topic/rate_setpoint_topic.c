#include "rate_setpoint_topic.h"
#include "Const.h"

static RateSetpoint_Data_t rateBuffer[2];

Topic_t rateSetpointTopic;

BaseType_t RateSetpointTopic_Init(void){
	return Topic_Init(&rateSetpointTopic, &rateBuffer[0], &rateBuffer[1], sizeof(RateSetpoint_Data_t));
}

BaseType_t RateSetpointTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&rateSetpointTopic, task, id);
}

BaseType_t RateSetpointTopic_Publish(const RateSetpoint_Data_t *rate){
	return Topic_Publish(&rateSetpointTopic, rate);
}

BaseType_t RateSetpointTopic_Copy(RateSetpoint_Data_t *rate){
	return Topic_Copy(&rateSetpointTopic, rate);
}
