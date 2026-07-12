#include "imu_topic.h"
#include "Const.h"

static BMI088_Data_t imuBuffer[2];

Topic_t imuTopic;

BaseType_t IMUTopic_Init(void){
	return Topic_Init(&imuTopic, &imuBuffer[0], &imuBuffer[1], sizeof(BMI088_Data_t));
}

BaseType_t IMUTopic_Subscribe(TaskHandle_t task, const uint8_t id){
	return Topic_Subscribe(&imuTopic, task, id);
}

BaseType_t IMUTopic_Publish(const BMI088_Data_t *imu){
	return Topic_Publish(&imuTopic, imu);
}

BaseType_t IMUTopic_Copy(BMI088_Data_t *imu){
	return Topic_Copy(&imuTopic, imu);
}
