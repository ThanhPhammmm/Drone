#include "rc_topic.h"

static RC_Data_t rcBuffer;
MutexTopic_t rcTopic;

BaseType_t RCTopic_Init(void){
	return MutexTopic_Init(&rcTopic, &rcBuffer, sizeof(RC_Data_t));
}

BaseType_t RCTopic_Publish(const RC_Data_t *rc){
	return MutexTopic_Publish(&rcTopic, rc);
}

BaseType_t RCTopic_Copy(RC_Data_t *rc, TickType_t timeout){
	return MutexTopic_Copy(&rcTopic, rc, timeout);
}
