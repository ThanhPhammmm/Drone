#include "range_topic.h"

static Range_Data_t rangeBuffer;
MutexTopic_t rangeTopic;

BaseType_t RangeTopic_Init(void){
	return MutexTopic_Init(&rangeTopic, &rangeBuffer, sizeof(Range_Data_t));
}

BaseType_t RangeTopic_Publish(const Range_Data_t *range){
	return MutexTopic_Publish(&rangeTopic, range);
}

BaseType_t RangeTopic_Copy(Range_Data_t *range, TickType_t timeout){
	return MutexTopic_Copy(&rangeTopic, range, timeout);
}
