#include "attitude_topic.h"
#include "Const.h"

static Attitude_Data_t attitudeBuffer[2];

Topic_t attitudeTopic;

BaseType_t AttitudeTopic_Init(void){
	return Topic_Init(&attitudeTopic, &attitudeBuffer[0], &attitudeBuffer[1], sizeof(Attitude_Data_t));
}

BaseType_t AttitudeTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&attitudeTopic, task, id);
}

BaseType_t AttitudeTopic_Publish(const Attitude_Data_t *att){
	return Topic_Publish(&attitudeTopic, att);
}

BaseType_t AttitudeTopic_Copy(Attitude_Data_t *att){
	return Topic_Copy(&attitudeTopic, att);
}
