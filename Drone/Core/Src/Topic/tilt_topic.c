#include "tilt_topic.h"
#include "Const.h"

static Tilt_Data_t tiltBuffer[2];

Topic_t tiltTopic;

BaseType_t TiltTopic_Init(void){
	return Topic_Init(&tiltTopic, &tiltBuffer[0], &tiltBuffer[1], sizeof(Tilt_Data_t));
}

BaseType_t TiltTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&tiltTopic, task, id);
}

BaseType_t TiltTopic_Publish(const Tilt_Data_t *tilt){
	return Topic_Publish(&tiltTopic, tilt);
}

BaseType_t TiltTopic_Copy(Tilt_Data_t *tilt){
	return Topic_Copy(&tiltTopic, tilt);
}
