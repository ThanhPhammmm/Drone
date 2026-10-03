#include "debug_task.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include "calibration.h"
#include "bmi088.h"
#include "bmp388.h"
#include "qmc5883.h"
#include "imu_topic.h"
#include "attitude_topic.h"
#include "attitude_setpoint_topic.h"
#include "rate_setpoint_topic.h"
#include "altitude_setpoint_topic.h"
#include "altitude_topic.h"
#include "thrust_topic.h"
#include "rc_topic.h"
#include "arm.h"
#include "flight_mode.h"
#include "actuator.h"
#include "rate_controller_task.h"

#define RAD2DEG		57.29578f

extern UART_HandleTypeDef huart1;
extern BMI088_Handle_t    bmi088;

static char dbgBuf[2048];
static int  dbgLen;

static void Dbg(const char *fmt, ...){
	if(dbgLen >= (int)sizeof(dbgBuf) - 1) return;

	va_list args;
	va_start(args, fmt);
	int n = vsnprintf(&dbgBuf[dbgLen], sizeof(dbgBuf) - (size_t)dbgLen, fmt, args);
	va_end(args);

	if(n < 0) return;
	dbgLen += n;
	if(dbgLen > (int)sizeof(dbgBuf) - 1) dbgLen = (int)sizeof(dbgBuf) - 1;	/* truncated */
}

static const char *ArmName(arm_state_t s){
	switch(s){
	case DISARMED:       return "DISARMED";
	case ARMING:         return "ARMING";
	case ARMED_GROUND:   return "ARMED_GROUND";
	case FLIGHT_TAKEOFF: return "TAKEOFF";
	case AIRBORNE:       return "AIRBORNE";
	case LANDING:        return "LANDING";
	case FAILSAFE:       return "FAILSAFE";
	default:             return "?";
	}
}

static const char *ModeName(uint8_t m){
	switch(m){
	case FLIGHT_MODE_ANGLE:    return "ANGLE";
	case FLIGHT_MODE_ALT_HOLD: return "ALT_HOLD";
	case FLIGHT_MODE_POS_HOLD: return "POS_HOLD";
	default:                   return "?";
	}
}

/* ------------------------------------------------------------------ */
/* calibration                                                         */
/* ------------------------------------------------------------------ */

static void Dbg_CalibState(void){
	static const char *name[CALIB_SENSOR_COUNT] = { "BMI088", "BMP388", "QMC5883" };

	Dbg("calib ");
	for(uint8_t s = 0; s < CALIB_SENSOR_COUNT; s++){
		Calib_Status_t st = Calib_GetStatus((Calib_Sensor_t)s);
		Dbg(" %s=%s", name[s], Calib_StatusName(st));
		if(st == CALIB_RUNNING){
			Dbg(" %.1fs", (float)Calib_ElapsedMs((Calib_Sensor_t)s) * 0.001f);
		}
	}
	Dbg(" | ready to arm: %s\r\n", Calib_ReadyToArm() ? "yes" : "no");

	/* what the person holding the drone has to do right now */
	if(Calib_GetStatus(CALIB_SENSOR_BMI088) == CALIB_RUNNING){
		uint8_t n = Calib_Attempts(CALIB_SENSOR_BMI088);
		Dbg("      -> BMI088: keep the drone STILL and LEVEL (~20 s), attempt %u%s\r\n",
		    n, (n > 1) ? " - the previous one moved, restarted" : "");
	}
	if(Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_RUNNING){
		Dbg("      -> BMP388: keep the drone on the ground (~2 s)\r\n");
	}
	if(Calib_GetStatus(CALIB_SENSOR_QMC5883) == CALIB_RUNNING){
		Dbg("      -> QMC5883: TURN the drone through every orientation (roll over, pitch over, spin flat) %lu/%lu s\r\n",
		    (unsigned long)(Calib_ElapsedMs(CALIB_SENSOR_QMC5883) / 1000U),
		    (unsigned long)(CALIB_QMC5883_DURATION_MS / 1000U));
	}
}

/* Each result once, in the form calibration.h expects. */
static uint8_t Dbg_CalibResults(uint8_t shown[CALIB_SENSOR_COUNT]){
	uint8_t added = 0;

	if(!shown[CALIB_SENSOR_BMI088] && Calib_GetStatus(CALIB_SENSOR_BMI088) == CALIB_DONE){
		Dbg("==== BMI088 calibrated: paste into calibration.h, then CALIB_BMI088_RUN 0 ====\r\n");
		Dbg("#define CALIB_BMI088_GYRO_BIAS_X \t%.6ff\r\n",  bmi088.calib.gyro_bias.x);
		Dbg("#define CALIB_BMI088_GYRO_BIAS_Y \t%.6ff\r\n",  bmi088.calib.gyro_bias.y);
		Dbg("#define CALIB_BMI088_GYRO_BIAS_Z \t%.6ff\r\n",  bmi088.calib.gyro_bias.z);
		Dbg("#define CALIB_BMI088_ACCEL_BIAS_X \t%.6ff\r\n", bmi088.calib.accel_bias.x);
		Dbg("#define CALIB_BMI088_ACCEL_BIAS_Y \t%.6ff\r\n", bmi088.calib.accel_bias.y);
		Dbg("#define CALIB_BMI088_ACCEL_BIAS_Z \t%.6ff\r\n", bmi088.calib.accel_bias.z);
		added |= 1U << CALIB_SENSOR_BMI088;
	}

	if(!shown[CALIB_SENSOR_BMP388] && Calib_GetStatus(CALIB_SENSOR_BMP388) == CALIB_DONE){
		Dbg("==== BMP388 calibrated: paste into calibration.h, then CALIB_BMP388_RUN 0 ====\r\n");
		Dbg("#define CALIB_BMP388_GROUND_PA \t\t%.1ff\r\n", bmp388.groundRef.pressure_pa);
		added |= 1U << CALIB_SENSOR_BMP388;
	}

	if(!shown[CALIB_SENSOR_QMC5883]){
		Calib_Status_t st = Calib_GetStatus(CALIB_SENSOR_QMC5883);
		if(st == CALIB_DONE){
			Dbg("==== QMC5883 calibrated: paste into calibration.h, then CALIB_QMC5883_RUN 0 ====\r\n");
			Dbg("#define CALIB_QMC5883_OFFSET_X \t\t%.1ff\r\n", qmc5883.calib.offset[0]);
			Dbg("#define CALIB_QMC5883_OFFSET_Y \t\t%.1ff\r\n", qmc5883.calib.offset[1]);
			Dbg("#define CALIB_QMC5883_OFFSET_Z \t\t%.1ff\r\n", qmc5883.calib.offset[2]);
			added |= 1U << CALIB_SENSOR_QMC5883;
		}
		else if(st == CALIB_FAILED){
			Dbg("==== QMC5883 calibration FAILED: not turned enough, compass off until next boot ====\r\n");
			Dbg("     span x=%.0f y=%.0f z=%.0f, each needs >= %.0f\r\n",
			    qmc5883.calib.maxV[0] - qmc5883.calib.minV[0],
			    qmc5883.calib.maxV[1] - qmc5883.calib.minV[1],
			    qmc5883.calib.maxV[2] - qmc5883.calib.minV[2],
			    QMC5883_CALIB_MIN_SPAN);
			added |= 1U << CALIB_SENSOR_QMC5883;
		}
	}
	return added;
}

/* ------------------------------------------------------------------ */
/* flight state                                                        */
/* ------------------------------------------------------------------ */

static void Dbg_Flight(void){
	BMI088_Data_t           imu  = {0};
	Attitude_Data_t         att  = {0};
	AttitudeSetpoint_Data_t asp  = {0};
	RateSetpoint_Data_t     rsp  = {0};
	AltitudeSetpoint_Data_t altSp = {0};
	Altitude_Data_t         alt  = {0};
	Thrust_Data_t           thr  = {0};
	RC_Data_t               rc   = {0};
	float torque[3];
	float pulse[MOTOR_COUNT];

	IMUTopic_Copy(&imu);
	AttitudeTopic_Copy(&att);
	AttitudeSetpointTopic_Copy(&asp);
	RateSetpointTopic_Copy(&rsp);
	AltitudeSetpointTopic_Copy(&altSp, 0);
	AltitudeTopic_Copy(&alt, 0);
	ThrustTopic_Copy(&thr, 0);
	RCTopic_Copy(&rc, 0);
	RateController_GetTorque(torque);
	MotorOutput_GetPulsesUs(pulse);

	Dbg("imu    gyro[dps] %+7.2f %+7.2f %+7.2f | acc[m/s2] %+6.2f %+6.2f %+6.2f | dt %.0f us\r\n",
	    imu.gyro.x * RAD2DEG, imu.gyro.y * RAD2DEG, imu.gyro.z * RAD2DEG,
	    imu.accel.x, imu.accel.y, imu.accel.z, imu.dt * 1e6f);

	Dbg("att    roll %+7.2f pitch %+7.2f yaw %+7.2f deg | rate[dps] %+7.2f %+7.2f %+7.2f | gyro bias[dps] %+.3f %+.3f %+.3f\r\n",
	    att.roll * RAD2DEG, att.pitch * RAD2DEG, att.yaw * RAD2DEG,
	    att.rollRate * RAD2DEG, att.pitchRate * RAD2DEG, att.yawRate * RAD2DEG,
	    att.gyroBiasX * RAD2DEG, att.gyroBiasY * RAD2DEG, att.gyroBiasZ * RAD2DEG);

	Dbg("sp     angle roll %+6.2f pitch %+6.2f deg, yawRate %+7.2f dps | rate sp[dps] %+7.2f %+7.2f %+7.2f\r\n",
	    asp.roll * RAD2DEG, asp.pitch * RAD2DEG, asp.yawRate * RAD2DEG,
	    rsp.rollRate * RAD2DEG, rsp.pitchRate * RAD2DEG, rsp.yawRate * RAD2DEG);

	Dbg("ctrl   torque %+.3f %+.3f %+.3f | thrust %.3f hold %u | climb sp %+.2f m/s manual %.2f\r\n",
	    torque[0], torque[1], torque[2], thr.thrust, thr.holdActive,
	    altSp.climbRate, altSp.manualThrust);

	Dbg("motor  us %4.0f %4.0f %4.0f %4.0f (M1 FR, M2 RR, M3 RL, M4 FL)\r\n",
	    pulse[0], pulse[1], pulse[2], pulse[3]);

	Dbg("rc     roll %+.2f pitch %+.2f yaw %+.2f thr %.2f\r\n",
	    rc.roll, rc.pitch, rc.yaw, rc.throttle);

	Dbg("alt    z %+7.2f m vz %+6.2f m/s accUp %+6.2f bias %+5.2f valid %u | baro %.1f Pa %.1f C %+7.2f m\r\n",
	    alt.altitude, alt.verticalSpeed, alt.accelUp, alt.accelBias, alt.valid,
	    bmp388.pressure_pa, bmp388.temperature_c, bmp388.altitude_m);

	/* heading only meaningful with the drone level and the compass calibrated */
	float mx = qmc5883.field.x, my = qmc5883.field.y, mz = qmc5883.field.z;
	float hdg = atan2f(-my, mx) * RAD2DEG;
	if(hdg < 0.0f) hdg += 360.0f;
	Dbg("mag    raw %6d %6d %6d | field[G] %+.3f %+.3f %+.3f |B| %.3f | heading(level) %5.1f deg\r\n",
	    qmc5883.raw.x, qmc5883.raw.y, qmc5883.raw.z,
	    mx, my, mz, sqrtf(mx*mx + my*my + mz*mz), hdg);
}

void DebugTask(void *argument){
	uint8_t shown[CALIB_SENSOR_COUNT] = {0};
	TickType_t last = xTaskGetTickCount();

	while(1){
		vTaskDelayUntil(&last, pdMS_TO_TICKS(DEBUG_PERIOD_MS));

		/* previous block still on the wire: skip this one rather than wait */
		if(huart1.gState != HAL_UART_STATE_READY) continue;

		RC_Data_t rc = {0};
		RCTopic_Copy(&rc, 0);

		dbgLen = 0;
		Dbg("\r\n---- t=%.1f s | arm=%s mode=%s link=%u\r\n",
		    (float)xTaskGetTickCount() * (float)portTICK_PERIOD_MS * 0.001f,
		    ArmName(Arm_GetState()), ModeName(rc.mode), rc.linkOk);
		Dbg_CalibState();
		uint8_t added = Dbg_CalibResults(shown);
		Dbg_Flight();

		if(HAL_UART_Transmit_DMA(&huart1, (uint8_t *)dbgBuf, (uint16_t)dbgLen) == HAL_OK){
			/* a dropped block shows the results again next time */
			for(uint8_t s = 0; s < CALIB_SENSOR_COUNT; s++){
				if(added & (1U << s)) shown[s] = 1;
			}
		}
	}
}
