#ifndef INC_SENSORDRIVER_BMI088_DRIVER_H_
#define INC_SENSORDRIVER_BMI088_DRIVER_H_

#include "main.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bmi088_data_types.h"

extern SPI_HandleTypeDef hspi1;

#define ACC_CS_GPIO_Port		GPIOA
#define ACC_CS_Pin				GPIO_PIN_4

#define GYRO_CS_GPIO_Port		GPIOA
#define GYRO_CS_Pin				GPIO_PIN_3

#define BMI088_GYRO_INT_Pin		GPIO_PIN_1 // PB1
#define BMI088_ACCEL_INT_Pin	GPIO_PIN_0 // PB0

#define BMI088_READ_BITMASK		0x80
#define BMI088_WRITE_BITMASK	0x7F

#define BMI088_TIMEOUT_MS     10

void BMI088_Port_Init(void);
void BMI088_AccCS_Low(void);
void BMI088_AccCS_High(void);
void BMI088_GyroCS_Low(void);
void BMI088_GyroCS_High(void);
HAL_StatusTypeDef BMI088_SPI_TransmitReceive_DMA(uint8_t *tx, uint8_t *rx, uint16_t len);
void BMI088_DMA_Callback(SPI_HandleTypeDef *hspi);
void BMI088_SetTaskHandle(TaskHandle_t handle);
BMI088_Status_t BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data);
BMI088_Status_t BMI088_Acc_WriteReg(uint8_t reg, uint8_t data);
BMI088_Status_t BMI088_Acc_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
BMI088_Status_t BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data);
BMI088_Status_t BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data);
BMI088_Status_t BMI088_Gyro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len);
void BMI088_ParseGyro(void);
void BMI088_ParseAccel(void);

#endif /* INC_SENSORDRIVER_BMI088_DRIVER_H_ */
