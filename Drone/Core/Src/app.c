#include "app.h"

void App_Init(void){
	/*
	 * Add inspections in the future
	 */

    IMUTopic_Init();
    AttitudeTopic_Init();
    RateSetpointTopic_Init();
    AttitudeSetpointTopic_Init();
    MagTopic_Init();
    BaroTopic_Init();
    AltitudeTopic_Init();
    ThrustTopic_Init();
    RCTopic_Init();
    AltitudeSetpointTopic_Init();

    Arm_Init();
}
