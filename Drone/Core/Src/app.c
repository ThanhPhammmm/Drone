#include "app.h"
#include "imu_topic.h"
#include "attitude_topic.h"
#include "rate_setpoint_topic.h"
#include "attitude_setpoint_topic.h"
#include "mag_topic.h"
#include "baro_topic.h"
#include "arm.h"
#include "motor_output.h"
#include "altitude_topic.h"

void App_Init(void){
    IMUTopic_Init();
    AttitudeTopic_Init();
    RateSetpointTopic_Init();
    AttitudeSetpointTopic_Init();
    MagTopic_Init();
    BaroTopic_Init();
    AltitudeTopic_Init();

    Arm_Init();
    MotorOutput_Init();
    //ESC_Calibrate(); /* Dont know why it hangs */
}
