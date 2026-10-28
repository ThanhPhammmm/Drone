#include "flow_topic.h"

static Flow_Data_t flowBuffer;
MutexTopic_t flowTopic;

BaseType_t FlowTopic_Init(void){
	return MutexTopic_Init(&flowTopic, &flowBuffer, sizeof(Flow_Data_t));
}

BaseType_t FlowTopic_Publish(const Flow_Data_t *flow){
	return MutexTopic_Publish(&flowTopic, flow);
}

BaseType_t FlowTopic_Copy(Flow_Data_t *flow, TickType_t timeout){
	return MutexTopic_Copy(&flowTopic, flow, timeout);
}
