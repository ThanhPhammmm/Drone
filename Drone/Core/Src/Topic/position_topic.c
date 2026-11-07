#include "position_topic.h"
#include "Const.h"

static Position_Data_t positionBuffer[2];

Topic_t positionTopic;

BaseType_t PositionTopic_Init(void){
	return Topic_Init(&positionTopic, &positionBuffer[0], &positionBuffer[1], sizeof(Position_Data_t));
}

BaseType_t PositionTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&positionTopic, task, id);
}

BaseType_t PositionTopic_Publish(const Position_Data_t *pos){
	return Topic_Publish(&positionTopic, pos);
}

BaseType_t PositionTopic_Copy(Position_Data_t *pos){
	return Topic_Copy(&positionTopic, pos);
}
