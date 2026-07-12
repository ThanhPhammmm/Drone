#include "app.h"
#include "imu_topic.h"
#include "attitude_topic.h"
#include "rate_setpoint_topic.h"
#include "attitude_setpoint_topic.h"
#include "arm.h"
#include "motor_output.h"

void App_Init(void){
    IMUTopic_Init();
    AttitudeTopic_Init();
    RateSetpointTopic_Init();
    AttitudeSetpointTopic_Init();

    Arm_Init();
    MotorOutput_Init();
}
