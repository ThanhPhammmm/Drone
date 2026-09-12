#ifndef INC_DEBUG_DEBUG_H_
#define INC_DEBUG_DEBUG_H_

#include "main.h"
#include "bmi088_task.h"
#include "bmi088.h"
#include "attitude_estimator_task.h"
#include "qmc5883_data_types.h"
#include "attitude_controller_task.h"
#include "string.h"
#include "stdio.h"
#include "rc_protocol.h"
#include "rc_topic.h"
#include "rate_controller_task.h"

void BMI088_PrintData(const uint32_t *gyro_count, const uint32_t *accel_count, const volatile uint32_t *imu_dt, const BMI088_Data_t *imu);
void BMI088_PrintDataCSV(const BMI088_Data_t *imu);
void BMI088_PrintAttitudeEstimator(const AttitudeEstimator_Handle_t* attitudeEstimator);
void BMI088_PrintAttitude(const Attitude_Data_t* attitude);
void QMC5883_PrintXYZ(const QMC5883_Raw_t* qmc5883);
void QMC5883_PrintXYZ_FIELD(const QMC5883_Field_t* qmc5883);
void Attitude_Controller_Print(const RateSetpoint_Data_t* rate);
void BMI088_Print_RawData(const BMI088_Data_t *imu);
void Motor_Print(uint8_t i, const uint32_t ccr);
void Motor_Setpoint_Print(RateSetpoint_Data_t* setpoint, const float thrust);
void RC_Print_Attitude_Setpoint(RC_Data_t* rc, float throttle);
void Motor_Torque_Print(RateController_Handle_t* torque, const float thrust);

#endif /* INC_DEBUG_DEBUG_H_ */
