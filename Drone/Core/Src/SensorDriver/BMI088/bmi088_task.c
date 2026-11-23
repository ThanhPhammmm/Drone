#include "bmi088_task.h"

extern BMI088_Handle_t bmi088;
extern volatile uint32_t gyro_dt;

SemaphoreHandle_t imuDmaSem;
#define GYRO_LPF_CUTOFF_HZ		80.0f	/* 80-100Hz */
#define GYRO_SAMPLE_RATE_HZ		1000.0f
#define ACCEL_SAMPLE_RATE_HZ	800.0f
#define ACCEL_LPF_CUTOFF_HZ		30.0f	/* 20-50Hz	*/

static BaseType_t IMU_Signal_Init(void) {
	imuDmaSem = xSemaphoreCreateBinary();
	return (imuDmaSem != NULL) ? pdPASS : pdFAIL;
}

static uint8_t IMU_CalibHook(uint32_t done, uint32_t total){
	CalSvc_Progress(RC_CAL_IMU, (uint8_t)(done * 100U / total), NULL);
	return CalSvc_CancelRequested(RC_CAL_IMU);
}

static int16_t IMU_Scaled(float v, float scale){
	float x = v * scale;
	if(x > 32767.0f) x = 32767.0f;
	if(x < -32767.0f) x = -32767.0f;
	return (int16_t)(x + (x >= 0.0f ? 0.5f : -0.5f));
}

/* Same measurement as at power-on (~20 s still). Moved or cancelled: the
 * values from before stay in use. */
static void IMU_RunCalibration(void){
	BMI088_Calib_t old = bmi088.calib;
	Calib_Status_t oldStatus = Calib_GetStatus(CALIB_SENSOR_BMI088);

	Calib_Start(CALIB_SENSOR_BMI088);
	BMI088_SetCalibHook(IMU_CalibHook);
	BMI088_Status_t st = BMI088_Calibrate(BMI088_CALIB_DEFAULT_SAMPLES);
	BMI088_SetCalibHook(NULL);

	if(st != BMI088_OK){
		uint8_t why = CalSvc_CancelRequested(RC_CAL_IMU) ? TLM_CAL_CANCELLED : TLM_CAL_MOVED;
		bmi088.calib = old;
		Calib_SetStatus(CALIB_SENSOR_BMI088, oldStatus);
		CalSvc_Finish(RC_CAL_IMU, why, NULL);
		return;
	}
	Calib_SetStatus(CALIB_SENSOR_BMI088, CALIB_DONE);

	int16_t r[6];
	r[0] = IMU_Scaled(bmi088.calib.gyro_bias.x, 1e6f);	/* 1e-6 rad/s */
	r[1] = IMU_Scaled(bmi088.calib.gyro_bias.y, 1e6f);
	r[2] = IMU_Scaled(bmi088.calib.gyro_bias.z, 1e6f);
	r[3] = IMU_Scaled(bmi088.calib.accel_bias.x, 1e4f);	/* 1e-4 m/s^2 */
	r[4] = IMU_Scaled(bmi088.calib.accel_bias.y, 1e4f);
	r[5] = IMU_Scaled(bmi088.calib.accel_bias.z, 1e4f);
	CalSvc_Finish(RC_CAL_IMU, TLM_CAL_OK, r);
}

void IMUTask(void *argument){
	if(IMU_Signal_Init() != pdPASS){
		Calib_SetStatus(CALIB_SENSOR_BMI088, CALIB_NO_SENSOR);
		vTaskDelete(NULL);
	}
	BMI088_SetTaskHandle(xTaskGetCurrentTaskHandle());
	if(BMI088_Init() != BMI088_OK){
		Calib_SetStatus(CALIB_SENSOR_BMI088, CALIB_NO_SENSOR);
		vTaskDelete(NULL);
	}

	static uint32_t lastGyroCycles = 0;
	static uint8_t gyroDtValid = 0;
	static LPF_t gyroLPF[3];
	static Biquad_t accelLPF[3];

	LPF_Init(&gyroLPF[0], GYRO_LPF_CUTOFF_HZ);
	LPF_Init(&gyroLPF[1], GYRO_LPF_CUTOFF_HZ);
	LPF_Init(&gyroLPF[2], GYRO_LPF_CUTOFF_HZ);

	Biquad_InitLowpass(&accelLPF[0], ACCEL_LPF_CUTOFF_HZ, ACCEL_SAMPLE_RATE_HZ);
	Biquad_InitLowpass(&accelLPF[1], ACCEL_LPF_CUTOFF_HZ, ACCEL_SAMPLE_RATE_HZ);
	Biquad_InitLowpass(&accelLPF[2], ACCEL_LPF_CUTOFF_HZ, ACCEL_SAMPLE_RATE_HZ);

#if CALIB_BMI088_RUN
	do{
		Calib_Start(CALIB_SENSOR_BMI088);
	}while(BMI088_Calibrate(BMI088_CALIB_DEFAULT_SAMPLES) != BMI088_OK);
	Calib_SetStatus(CALIB_SENSOR_BMI088, CALIB_DONE);
#else
	BMI088_SetCalibration(CALIB_BMI088_GYRO_BIAS_X, CALIB_BMI088_GYRO_BIAS_Y, CALIB_BMI088_GYRO_BIAS_Z,
						  CALIB_BMI088_ACCEL_BIAS_X, CALIB_BMI088_ACCEL_BIAS_Y, CALIB_BMI088_ACCEL_BIAS_Z);
	Calib_SetStatus(CALIB_SENSOR_BMI088, CALIB_STORED);
#endif
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if(CalSvc_Take(RC_CAL_IMU)){
			IMU_RunCalibration();
			gyroDtValid = 0;	/* the next sample's dt would span the ~20 s pause */
			continue;
		}
		bmi088.data.gyro_updated = false;
		bmi088.data.accel_updated = false;

		if(bmi088.gyroReady){
			bmi088.gyroReady = false;
			uint32_t sampleCycles = bmi088.gyroIrqCycles;
			if(BMI088_ReadGyro() != BMI088_OK) continue;
			BMI088_ParseGyro();
			bmi088.data.gyro_updated = true;

			bmi088.data.dt = gyroDtValid ? (float)(sampleCycles - lastGyroCycles) / (float)SystemCoreClock : 0.0f;
			gyroDtValid = 1;
			lastGyroCycles = sampleCycles;

			bmi088.data.timestamp_us = Time_UsAt(sampleCycles);
			bmi088.data.timestamp = xTaskGetTickCount();
		}
		if(bmi088.accelReady){
			bmi088.accelReady = false;
			if(BMI088_ReadAccel() != BMI088_OK) continue;
			BMI088_ParseAccel();
			bmi088.data.accel_updated = true;
		}

		//BMI088_Convert(); Comment for now

		if(bmi088.data.gyro_updated){
			BMI088_Gyro_Convert();
		}
		if(bmi088.data.accel_updated){
			BMI088_Accel_Convert();
		}

		if(bmi088.data.gyro_updated){
			bmi088.data.gyro.x = LPF_Update(&gyroLPF[0], bmi088.data.gyro.x, bmi088.data.dt);
			bmi088.data.gyro.y = LPF_Update(&gyroLPF[1], bmi088.data.gyro.y, bmi088.data.dt);
			bmi088.data.gyro.z = LPF_Update(&gyroLPF[2], bmi088.data.gyro.z, bmi088.data.dt);
		}
		if(bmi088.data.accel_updated){
			bmi088.data.accel.x = Biquad_Update(&accelLPF[0], bmi088.data.accel.x);
			bmi088.data.accel.y = Biquad_Update(&accelLPF[1], bmi088.data.accel.y);
			bmi088.data.accel.z = Biquad_Update(&accelLPF[2], bmi088.data.accel.z);
		}
		//publish to topic
		if(bmi088.data.gyro_updated){
			IMUTopic_Publish(&bmi088.data);
		}
	}
}
