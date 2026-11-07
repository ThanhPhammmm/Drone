#include "app.h"
#include "i2c.h"

static DMA_HandleTypeDef i2cNoTxDma;

static void App_I2CNoTxDma(void){
	i2cNoTxDma.State = HAL_DMA_STATE_READY;
	hi2c1.hdmatx = &i2cNoTxDma;
	hi2c2.hdmatx = &i2cNoTxDma;
}

void App_Init(void){
	/*
	 * Add inspections in the future
	 */

	App_I2CNoTxDma();

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
    RangeTopic_Init();
    FlowTopic_Init();
    GpsTopic_Init();
    PositionTopic_Init();
    TiltTopic_Init();
    Arm_Init();
}
