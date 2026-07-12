#include "bmi088.h"
#include "bmi088_reg.h"
#include "bmi088_port.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

static uint8_t gyroTx[8];
static uint8_t gyroRx[8];

static uint8_t accelTx[8];
static uint8_t accelRx[8];

BMI088_Handle_t bmi088;
extern SemaphoreHandle_t imuDmaSem;

static float accSensitivity = 24.0f / 32768.0f;
static float gyroSensitivity = 2000.0f / 32768.0f;

static void BMI088_EnableSPI(void){
    uint8_t dummy;

    BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID_REG, &dummy);
    vTaskDelay(pdMS_TO_TICKS(1));
}

static BMI088_Status_t BMI088_Reset(void){
    BMI088_Status_t status;

    status = BMI088_Acc_WriteReg(BMI088_ACC_SOFTRESET, BMI088_SOFT_RESET_CMD);
    if(status != BMI088_OK) return status;

    vTaskDelay(pdMS_TO_TICKS(50));

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_SOFTRESET, BMI088_SOFT_RESET_CMD);
    if(status != BMI088_OK) return status;

    vTaskDelay(pdMS_TO_TICKS(50));

    return BMI088_OK;
}

static BMI088_Status_t BMI088_CheckChip(void){
    uint8_t acc;
    uint8_t gyro;

    BMI088_Acc_ReadReg(BMI088_ACC_CHIP_ID_REG, &acc);
    if(acc != BMI088_ACC_CHIP_ID) return BMI088_INVALID_CHIP;
    BMI088_Gyro_ReadReg(BMI088_GYRO_CHIP_ID_REG, &gyro);
    if(gyro != BMI088_GYRO_CHIP_ID) return BMI088_INVALID_CHIP;

    return BMI088_OK;
}

static BMI088_Status_t BMI088_ConfigAccel(void)
{
    BMI088_Status_t status;

    status = BMI088_Acc_WriteReg(BMI088_ACC_RANGE, BMI088_ACC_RANGE_24G);
    if(status != BMI088_OK) return status;

    status = BMI088_Acc_WriteReg(BMI088_ACC_CONF, BMI088_ACC_ODR_1600);
    if(status != BMI088_OK) return status;

    return status;
}

static BMI088_Status_t BMI088_ConfigGyro(void){
    BMI088_Status_t status;

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_RANGE, BMI088_GYRO_RANGE_2000);
    if(status != BMI088_OK) return status;

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_BANDWIDTH, BMI088_GYRO_ODR_2000);
    if(status != BMI088_OK) return status;

    return status;
}

static BMI088_Status_t BMI088_ConfigAccelInterrupt(void){
    BMI088_Status_t status;

    status = BMI088_Acc_WriteReg(BMI088_ACC_INT1_IO_CTRL, 0x0A); // PushPull + Active high
    if(status != BMI088_OK) return status;

    status = BMI088_Acc_WriteReg(BMI088_ACC_INT_MAP_DATA, 0x04);
    if(status != BMI088_OK) return status;

    return status;
}

static BMI088_Status_t BMI088_ConfigGyroInterrupt(void){
    BMI088_Status_t status;

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT_CTRL, 0x80);
    if(status != BMI088_OK) return status;

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_CONF, 0x01); // PushPull + Active High
    if(status != BMI088_OK) return status;

    status = BMI088_Gyro_WriteReg(BMI088_GYRO_INT3_INT4_IO_MAP, 0x01);
    if(status != BMI088_OK) return status;

    return status;
}

BMI088_Status_t BMI088_Init(void){
    BMI088_Status_t status;

    BMI088_EnableSPI();

    status = BMI088_Reset();
    if(status != BMI088_OK) return status;

    BMI088_EnableSPI();

    status = BMI088_CheckChip();
    if(status != BMI088_OK) return status;

    status = BMI088_ConfigAccel();
    if(status != BMI088_OK) return status;

    status = BMI088_ConfigGyro();
    if(status != BMI088_OK) return status;

    status = BMI088_ConfigAccelInterrupt();
    if(status != BMI088_OK) return status;

    status = BMI088_ConfigGyroInterrupt();
    if(status != BMI088_OK) return status;

    return BMI088_OK;
}

BMI088_Status_t BMI088_ReadGyro(void){
    gyroTx[0] = BMI088_GYRO_RATE_X_LSB | BMI088_READ_BITMASK;

    memset(gyroTx + 1, 0, sizeof(gyroTx) - 1);
    memset(gyroRx, 0, sizeof(gyroRx));
    BMI088_GyroCS_Low();
    while(xSemaphoreTake(imuDmaSem, 0) == pdTRUE);
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, gyroTx, gyroRx, 7) != HAL_OK){
        BMI088_GyroCS_High();
        return BMI088_ERROR;
    }
    if(xSemaphoreTake(imuDmaSem, pdMS_TO_TICKS(5)) == 0){
        BMI088_GyroCS_High();
        return BMI088_TIMEOUT;
    }
    BMI088_GyroCS_High();

    return BMI088_OK;
}

void BMI088_ParseGyro(void){
    bmi088.data.gyro_raw.x = (int16_t)((gyroRx[2] << 8) | gyroRx[1]);
    bmi088.data.gyro_raw.y = (int16_t)((gyroRx[4] << 8) | gyroRx[3]);
    bmi088.data.gyro_raw.z = (int16_t)((gyroRx[6] << 8) | gyroRx[5]);
}

BMI088_Status_t BMI088_ReadAccel(void){
    accelTx[0] = BMI088_ACC_X_LSB | BMI088_READ_BITMASK;

    memset(accelTx + 1, 0, sizeof(accelTx) - 1);
    memset(accelRx, 0, sizeof(accelRx));
    BMI088_AccCS_Low();
    while(xSemaphoreTake(imuDmaSem, 0) == pdTRUE);
    if(HAL_SPI_TransmitReceive_DMA(&hspi1, accelTx, accelRx, 8) != HAL_OK){
        BMI088_AccCS_High();
        return BMI088_ERROR;
    }
    if(xSemaphoreTake(imuDmaSem, pdMS_TO_TICKS(5)) == 0){
        BMI088_AccCS_High();
        return BMI088_TIMEOUT;
    }
    BMI088_AccCS_High();

    return BMI088_OK;
}

void BMI088_ParseAccel(void){
    bmi088.data.accel_raw.x = (int16_t)((accelRx[3] << 8) | accelRx[2]);
    bmi088.data.accel_raw.y = (int16_t)((accelRx[5] << 8) | accelRx[4]);
    bmi088.data.accel_raw.z = (int16_t)((accelRx[7] << 8) | accelRx[6]);
}

void BMI088_ParseData(void){
	BMI088_ParseGyro();
	BMI088_ParseAccel();
}

void BMI088_Convert(void){
    bmi088.data.accel.x =
        bmi088.data.accel_raw.x *
        accSensitivity *
        BMI088_G;

    bmi088.data.accel.y =
        bmi088.data.accel_raw.y *
        accSensitivity *
        BMI088_G;

    bmi088.data.accel.z =
        bmi088.data.accel_raw.z *
        accSensitivity *
        BMI088_G;

    bmi088.data.gyro.x =
        bmi088.data.gyro_raw.x *
        gyroSensitivity *
        DEG2RAD;

    bmi088.data.gyro.y =
        bmi088.data.gyro_raw.y *
        gyroSensitivity *
        DEG2RAD;

    bmi088.data.gyro.z =
        bmi088.data.gyro_raw.z *
        gyroSensitivity *
        DEG2RAD;
}

