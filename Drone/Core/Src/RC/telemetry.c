#include "telemetry.h"

#define TLM_RAD2DEG             57.29578f
#define TLM_ACCEL_G_PER_LSB     (12.0f / 32768.0f)

static int16_t TLM_S16(float v){
	v = (v >= 0.0f) ? v + 0.5f : v - 0.5f;
	if(v >  32767.0f) return  32767;
	if(v < -32767.0f) return -32767;
	return (int16_t)v;
}

static uint16_t TLM_U16(float v){
	v += 0.5f;
	if(v > 65535.0f) return 65535;
	if(v < 0.0f)     return 0;
	return (uint16_t)v;
}

static uint8_t TLM_State(void){
	uint8_t s = (uint8_t)Arm_GetState() & TLM_STATE_ARM_MASK;
	s |= ((uint8_t)FlightMode_GetActive() << TLM_STATE_MODE_SHIFT) & TLM_STATE_MODE_MASK;
	if(Calib_Usable(CALIB_SENSOR_BMI088))  s |= TLM_STATE_IMU_CAL;
	if(Calib_Usable(CALIB_SENSOR_BMP388))  s |= TLM_STATE_BARO_CAL;
	if(Calib_Usable(CALIB_SENSOR_QMC5883)) s |= TLM_STATE_MAG_CAL;
	return s;
}

static uint8_t TLM_BuildAttitude(uint8_t *buf, uint8_t rcSeq, uint8_t state){
	Attitude_Data_t         att = {0};
	AttitudeSetpoint_Data_t asp = {0};
	RateSetpoint_Data_t     rsp = {0};
	float torque[3];

	AttitudeTopic_Copy(&att);
	AttitudeSetpointTopic_Copy(&asp);
	RateSetpointTopic_Copy(&rsp);
	RateController_GetTorque(torque);

	TLM_Attitude_t f;
	f.magic     = TLM_MAGIC;
	f.type      = TLM_FRAME_ATTITUDE;
	f.rcSeq     = rcSeq;
	f.state     = state;
	f.roll      = TLM_S16(att.roll  * TLM_RAD2DEG * 100.0f);
	f.pitch     = TLM_S16(att.pitch * TLM_RAD2DEG * 100.0f);
	f.yaw       = TLM_S16(att.yaw   * TLM_RAD2DEG * 100.0f);
	f.rollSp    = TLM_S16(asp.roll  * TLM_RAD2DEG * 100.0f);
	f.pitchSp   = TLM_S16(asp.pitch * TLM_RAD2DEG * 100.0f);
	f.rate[0]   = TLM_S16(att.rollRate  * TLM_RAD2DEG * 10.0f);
	f.rate[1]   = TLM_S16(att.pitchRate * TLM_RAD2DEG * 10.0f);
	f.rate[2]   = TLM_S16(att.yawRate   * TLM_RAD2DEG * 10.0f);
	f.rateSp[0] = TLM_S16(rsp.rollRate  * TLM_RAD2DEG * 10.0f);
	f.rateSp[1] = TLM_S16(rsp.pitchRate * TLM_RAD2DEG * 10.0f);
	f.rateSp[2] = TLM_S16(rsp.yawRate   * TLM_RAD2DEG * 10.0f);
	f.torque[0] = TLM_S16(torque[0] * 10000.0f);
	f.torque[1] = TLM_S16(torque[1] * 10000.0f);
	f.torque[2] = TLM_S16(torque[2] * 10000.0f);

	memcpy(buf, &f, sizeof(f));
	return (uint8_t)sizeof(f);
}

static uint8_t TLM_BuildOutput(uint8_t *buf, uint8_t rcSeq, uint8_t state, uint16_t rcLost,
                               const Altitude_Data_t *alt, uint8_t altValid){
	AltitudeSetpoint_Data_t altSp = {0};
	Baro_Data_t   baro = {0};
	Thrust_Data_t thr  = {0};
	BMI088_Data_t imu  = {0};

	AltitudeSetpointTopic_Copy(&altSp, 0);
	BaroTopic_Copy(&baro, 0);
	ThrustTopic_Copy(&thr, 0);
	IMUTopic_Copy(&imu);

	MotorSaturation_t sat = RateController_GetSaturation();

	float ax = (float)imu.accel_raw.x * TLM_ACCEL_G_PER_LSB;
	float ay = (float)imu.accel_raw.y * TLM_ACCEL_G_PER_LSB;
	float az = (float)imu.accel_raw.z * TLM_ACCEL_G_PER_LSB;
	float vib = fabsf(sqrtf(ax*ax + ay*ay + az*az) - 1.0f) * BMI088_G * 10.0f;

	float motor[MOTOR_COUNT];
	MotorOutput_GetPulsesUs(motor);

	TLM_Output_t f;
	f.magic        = TLM_MAGIC;
	f.type         = TLM_FRAME_OUTPUT;
	f.rcSeq        = rcSeq;
	f.state        = state;
	for(uint8_t i = 0; i < MOTOR_COUNT; i++) f.motor[i] = motor[i];
	f.thrust       = TLM_U16(thr.thrust * 10000.0f);
	f.altitude     = TLM_S16(alt->altitude * 100.0f);
	f.baroAltitude = TLM_S16(baro.altitude_m * 100.0f);
	f.vz           = TLM_S16(alt->verticalSpeed * 100.0f);
	f.climbSp      = TLM_S16(altSp.climbRate * 100.0f);
	f.accelUp      = TLM_S16(alt->accelUp * 100.0f);
	f.rateDtMaxUs  = TLM_U16((float)RateController_TakeMaxStepUs());
	f.rcLost       = rcLost;
	f.hoverThrust  = TLM_U16(thr.hoverThrust * 10000.0f);
	f.sat          = (sat.rollPitchSaturated    ? TLM_SAT_ROLL_PITCH : 0) |
	                 (sat.yawSaturated          ? TLM_SAT_YAW        : 0) |
	                 (sat.throttleSaturatedHigh ? TLM_SAT_THR_HIGH   : 0) |
	                 (sat.throttleSaturatedLow  ? TLM_SAT_THR_LOW    : 0) |
	                 (Arm_Tumbled()             ? TLM_SAT_TUMBLE     : 0) |
	                 (altValid                  ? TLM_SAT_ALT_VALID  : 0);
	f.vibration    = (uint8_t)((vib > 255.0f) ? 255.0f : vib + 0.5f);

	memcpy(buf, &f, sizeof(f));
	return (uint8_t)sizeof(f);
}

uint8_t Telemetry_Build(uint8_t *buf, uint8_t rcSeq, uint16_t rcLost){
	static uint8_t next = TLM_FRAME_ATTITUDE;

	Altitude_Data_t alt = {0};
	uint8_t altValid = (AltitudeTopic_Copy(&alt, 0) == pdPASS) && alt.valid;
	uint8_t state    = TLM_State();

	uint8_t len;
	if(next == TLM_FRAME_ATTITUDE){
		len  = TLM_BuildAttitude(buf, rcSeq, state);
		next = TLM_FRAME_OUTPUT;
	}
	else{
		len  = TLM_BuildOutput(buf, rcSeq, state, rcLost, &alt, altValid);
		next = TLM_FRAME_ATTITUDE;
	}
	return len;
}