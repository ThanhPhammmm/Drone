#ifndef INC_SENSORDRIVER_BMI088_BMI088_H_
#define INC_SENSORDRIVER_BMI088_BMI088_H_

#include "bmi088_port.h"
#include "bmi088_data_types.h"

#define BMI088_G			9.80665f
#define DEG2RAD				0.017453292519943295f

typedef struct{
    BMI088_Data_t data;
    TaskHandle_t imuTask;
    uint8_t gyroReady;
    uint8_t accelReady;
} BMI088_Handle_t;

BMI088_Status_t BMI088_Init(void);
BMI088_Status_t BMI088_Start(void);
BMI088_Status_t BMI088_Stop(void);
BMI088_Status_t BMI088_ReadAccel(void);
BMI088_Status_t BMI088_ReadGyro(void);
BMI088_Status_t BMI088_GetData(BMI088_Data_t *imu);
void BMI088_ParseData(void);
void BMI088_Convert(void);

#endif /* INC_SENSORDRIVER_BMI088_BMI088_H_ */
