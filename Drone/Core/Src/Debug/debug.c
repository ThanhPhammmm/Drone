#include "debug.h"

extern UART_HandleTypeDef huart1;

void BMI088_PrintData(const uint32_t *gyro_count, const uint32_t *accel_count, const volatile uint32_t *imu_dt, const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 500)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"gyro_count = %lu, accel_count = %lu, dt = %lu - ACC: %.3f %.3f %.3f | GYR: %.3f %.3f %.3f\r\n",
		*gyro_count, *accel_count, *imu_dt,
		imu->accel.x, imu->accel.y, imu->accel.z,
		imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void BMI088_PrintDataCSV(const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%lu\r\n"
		"accel: %.6f, %.6f, %.6f\r\n"
		"gyro: %.6f, %.6f, %.6f\r\n",
		imu->timestamp_us,
		imu->accel.x, imu->accel.y, imu->accel.z,
		imu->gyro.x,  imu->gyro.y,  imu->gyro.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void BMI088_Print_RawData(const BMI088_Data_t *imu){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%lu\r\n"
		"accel: %.d, %.d, %.d\r\n"
		"gyro: %.d, %.d, %.d\r\n",
		imu->timestamp_us,
		imu->accel_raw.x, imu->accel_raw.y, imu->accel_raw.z,
		imu->gyro_raw.x,  imu->gyro_raw.y,  imu->gyro_raw.z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void BMI088_PrintAttitudeEstimator(const AttitudeEstimator_Handle_t* attitudeEstimator){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 20)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f,%.6f,%.6f\r\n",
		attitudeEstimator->data.roll,attitudeEstimator->data.pitch, attitudeEstimator->data.yaw);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void BMI088_PrintAttitude(const Attitude_Data_t* attitude){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf), "estimator: roll: %.6f, pitch: %.6f, yaw: %.6f\r\n", attitude->roll,attitude->pitch, attitude->yaw);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void QMC5883_PrintXYZ(const QMC5883_Raw_t* qmc5883){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.d,%.d,%.d\r\n",
		qmc5883->x,qmc5883->y, qmc5883->z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void QMC5883_PrintXYZ_FIELD(const QMC5883_Field_t* qmc5883){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f,%.6f,%.6f\r\n",
		qmc5883->x,qmc5883->y, qmc5883->z);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void Attitude_Controller_Print(const RateSetpoint_Data_t* rate){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"rate: roll: %.6f, pitch: %.6f, yaw: %.6f\r\n", rate->rollRate,rate->pitchRate, rate->yawRate);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

uint32_t current_time[4] = {0};
uint32_t last_print_time[4] = {0};

void Motor_Print(uint8_t i, const uint32_t ccr){
	static char buf[128];
	current_time[i] = HAL_GetTick();

	if (current_time[i] - last_print_time[i] < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf), "motor %d: %ld\r\n", i, ccr);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time[i] = current_time[i];
	}
}

void Motor_Setpoint_Print(RateSetpoint_Data_t* setpoint, const float thrust){
	static char buf[128];
	uint32_t current_time = HAL_GetTick();
	static uint32_t last_print_time = 0;

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f, %.6f, %.6f, %.6f\r\n",
		setpoint->rollRate,setpoint->pitchRate, setpoint->yawRate, thrust);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void RC_Print_Attitude_Setpoint(RC_Data_t* rc, float throttle){
	static char buf[128];
	static uint32_t last_print_time = 0;
	uint32_t current_time = HAL_GetTick();

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"setpoint: roll: %.6f, pitch: %.6f, yaw: %.6f, throttle: %.6f\r\n",
		rc->roll,  rc->pitch,  rc->yaw, throttle);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}

void Motor_Torque_Print(RateController_Handle_t* torque, const float thrust){
	static char buf[128];
	uint32_t current_time = HAL_GetTick();
	static uint32_t last_print_time = 0;

	if (current_time - last_print_time < 1000)
		return;

	if (huart1.gState != HAL_UART_STATE_READY)
		return;

	int len = snprintf(buf, sizeof(buf),
		"%.6f, %.6f, %.6f, %.6f\r\n", torque->rollTorqueOutput,torque->pitchTorqueOutput, torque->yawTorqueOutput, thrust);

	if (HAL_UART_Transmit_DMA(&huart1, (uint8_t *)buf, len) == HAL_OK){
		last_print_time = current_time;
	}
}
