#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "bmi088_port.h"
#include "bmi088.h"
#include "semphr.h"

#define BMI088_TIMEOUT_MS     10

uint8_t txBuf[32];
uint8_t rxBuf[32];

extern SPI_HandleTypeDef hspi1;
extern BMI088_Handle_t bmi088;
extern SemaphoreHandle_t imuDmaSem;

void BMI088_Port_Init(void){
	bmi088.imuTask = NULL;
}

void BMI088_SetTaskHandle(TaskHandle_t handle){
	bmi088.imuTask = handle;
}

void BMI088_AccCS_Low(void){
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_AccCS_High(void){
    HAL_GPIO_WritePin(ACC_CS_GPIO_Port, ACC_CS_Pin, GPIO_PIN_SET);
}

void BMI088_GyroCS_Low(void){
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET);
}

void BMI088_GyroCS_High(void){
    HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef BMI088_SPI_TransmitReceive_DMA(uint8_t *tx, uint8_t *rx, uint16_t len){
    return HAL_SPI_TransmitReceive_DMA(&hspi1, tx, rx, len);
}

void BMI088_DMA_Callback(SPI_HandleTypeDef *hspi){
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(imuDmaSem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static BMI088_Status_t BMI088_WaitDMA(void){
    if(xSemaphoreTake(imuDmaSem, pdMS_TO_TICKS(BMI088_TIMEOUT_MS)) != pdTRUE){
        return BMI088_TIMEOUT;
    }

    return BMI088_OK;
}

static void BMI088_ClearDMACompletion(void){
    while(xSemaphoreTake(imuDmaSem, 0) == pdTRUE){}
}

BMI088_Status_t BMI088_Acc_WriteReg(uint8_t reg, uint8_t data){
    txBuf[0] = reg & BMI088_WRITE_BITMASK;
    txBuf[1] = data;

    BMI088_ClearDMACompletion();
    BMI088_AccCS_Low();
    HAL_StatusTypeDef ret = BMI088_SPI_TransmitReceive_DMA(txBuf, rxBuf, 2);
    if(ret != HAL_OK){
        BMI088_AccCS_High();
        return BMI088_ERROR;
    }
    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_AccCS_High();

    return status;
}

BMI088_Status_t BMI088_Acc_ReadReg(uint8_t reg, uint8_t *data){
    txBuf[0] = reg | BMI088_READ_BITMASK;
    txBuf[1] = 0;
    txBuf[2] = 0;

    BMI088_ClearDMACompletion();
    BMI088_AccCS_Low();
    HAL_StatusTypeDef ret = BMI088_SPI_TransmitReceive_DMA(txBuf, rxBuf, 3);
    if(ret != HAL_OK){
        BMI088_AccCS_High();
        return BMI088_ERROR;
    }
    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_AccCS_High();
    if(status != BMI088_OK) return status;
    *data = rxBuf[2];

    return BMI088_OK;
}

BMI088_Status_t BMI088_Acc_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len){
    txBuf[0] = reg | BMI088_READ_BITMASK;

    memset(&txBuf[1], 0, len + 1); /* dummy byte + register data */
    BMI088_ClearDMACompletion();
    BMI088_AccCS_Low();
    HAL_StatusTypeDef ret = BMI088_SPI_TransmitReceive_DMA(txBuf, rxBuf, len + 2);
    if(ret != HAL_OK){
    	BMI088_AccCS_High();
        return BMI088_ERROR;
    }

    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_AccCS_High();
    if(status != BMI088_OK) return status;
    memcpy(data, &rxBuf[2], len);

    return BMI088_OK;
}

BMI088_Status_t BMI088_Gyro_WriteReg(uint8_t reg, uint8_t data){
    txBuf[0] = reg & BMI088_WRITE_BITMASK;
    txBuf[1] = data;

    BMI088_ClearDMACompletion();
    BMI088_GyroCS_Low();
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, txBuf, rxBuf, 2) != HAL_OK){
        BMI088_GyroCS_High();
        return BMI088_ERROR;
    }
    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_GyroCS_High();

    return status;
}

BMI088_Status_t BMI088_Gyro_ReadReg(uint8_t reg, uint8_t *data){
    txBuf[0] = reg | BMI088_READ_BITMASK;
    txBuf[1] = 0;

    BMI088_ClearDMACompletion();
    BMI088_GyroCS_Low();
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, txBuf, rxBuf, 2) != HAL_OK){
        BMI088_GyroCS_High();
        return BMI088_ERROR;
    }
    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_GyroCS_High();
    if(status != BMI088_OK) return status;

    *data = rxBuf[1];

    return BMI088_OK;
}

BMI088_Status_t BMI088_Gyro_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len){
    txBuf[0] = reg | BMI088_READ_BITMASK;

    memset(&txBuf[1], 0, len);
    BMI088_ClearDMACompletion();
    BMI088_GyroCS_Low();
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, txBuf, rxBuf, len + 1) != HAL_OK){
        BMI088_GyroCS_High();
        return BMI088_ERROR;
    }
    BMI088_Status_t status = BMI088_WaitDMA();
    BMI088_GyroCS_High();
    if(status != BMI088_OK) return status;
    memcpy(data, &rxBuf[1], len);

    return BMI088_OK;
}
