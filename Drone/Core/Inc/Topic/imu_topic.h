#ifndef INC_TOPIC_IMU_TOPIC_H_
#define INC_TOPIC_IMU_TOPIC_H_

#include "topic.h"
#include "bmi088.h"

extern Topic_t imuTopic;

BaseType_t IMUTopic_Init(void);
BaseType_t IMUTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t IMUTopic_Publish(const BMI088_Data_t *imu);
BaseType_t IMUTopic_Copy(BMI088_Data_t *imu);

#endif /* INC_TOPIC_IMU_TOPIC_H_ */
