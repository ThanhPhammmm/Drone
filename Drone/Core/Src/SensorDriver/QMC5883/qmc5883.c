#include "qmc5883.h"
#include "qmc5883_reg.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#define MAG_I2C						hi2c2
#define QMC5883_I2C_TIMEOUT_MS		10

QMC5883_Handle_t qmc5883;

static QMC5883_Status_t QMC5883_WriteReg(uint8_t reg, uint8_t val){
    if(HAL_I2C_Mem_Write(&MAG_I2C, QMC5883_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, QMC5883_I2C_TIMEOUT_MS) != HAL_OK){
        return QMC5883_ERROR;
    }
    return QMC5883_OK;
}

static QMC5883_Status_t QMC5883_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len){
    if(HAL_I2C_Mem_Read(&MAG_I2C, QMC5883_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                         buf, len, QMC5883_I2C_TIMEOUT_MS) != HAL_OK){
        return QMC5883_ERROR;
    }
    return QMC5883_OK;
}

QMC5883_Status_t QMC5883_Init(void){
    uint8_t chipId = 0;
    if(QMC5883_ReadRegs(QMC5883_REG_CHIP_ID, &chipId, 1) != QMC5883_OK) return QMC5883_ERROR;
    if(chipId != QMC5883_CHIP_ID_VALUE) return QMC5883_INVALID_CHIP;

    if(QMC5883_WriteReg(QMC5883_REG_CTRL2, QMC5883_CTRL2_SOFT_RST) != QMC5883_OK) return QMC5883_ERROR;
    vTaskDelay(pdMS_TO_TICKS(10));

    if(QMC5883_WriteReg(QMC5883_REG_SIGN, 0x06) != QMC5883_OK) return QMC5883_ERROR;

    uint8_t ctrl2 = QMC5883_RNG_8G | QMC5883_SETRESET_ON;
    if(QMC5883_WriteReg(QMC5883_REG_CTRL2, ctrl2) != QMC5883_OK) return QMC5883_ERROR;

    uint8_t ctrl1 = QMC5883_OSR2_8 | QMC5883_OSR1_8 | QMC5883_ODR_100HZ | QMC5883_MODE_CONTINUOUS;
    if(QMC5883_WriteReg(QMC5883_REG_CTRL1, ctrl1) != QMC5883_OK) return QMC5883_ERROR;

    qmc5883.calib.offset[0] = 0.0f;
    qmc5883.calib.offset[1] = 0.0f;
    qmc5883.calib.offset[2] = 0.0f;
    qmc5883.calib.calibrated = 0;

    return QMC5883_OK;
}

QMC5883_Status_t QMC5883_Read(void){
    uint8_t buf[6];
    if(QMC5883_ReadRegs(QMC5883_REG_DATA_X_LSB, buf, 6) != QMC5883_OK) return QMC5883_ERROR;

    qmc5883.raw.x = (int16_t)((buf[1] << 8) | buf[0]);
    qmc5883.raw.y = (int16_t)((buf[3] << 8) | buf[2]);
    qmc5883.raw.z = (int16_t)((buf[5] << 8) | buf[4]);

    qmc5883.field.x = ((float)qmc5883.raw.x - qmc5883.calib.offset[0]) / QMC5883_LSB_PER_GAUSS_8G;
    qmc5883.field.y = ((float)qmc5883.raw.y - qmc5883.calib.offset[1]) / QMC5883_LSB_PER_GAUSS_8G;
    qmc5883.field.z = ((float)qmc5883.raw.z - qmc5883.calib.offset[2]) / QMC5883_LSB_PER_GAUSS_8G;

    return QMC5883_OK;
}

void QMC5883_CalibReset(void){
    for(uint8_t k = 0; k < 3; k++){
        qmc5883.calib.minV[k]	=  1e6f;
        qmc5883.calib.maxV[k]	= -1e6f;
        qmc5883.calib.offset[k]	=  0.0f;
    }
    qmc5883.calib.calibrated = 0;
}

void QMC5883_CalibAccumulate(void){
    float s[3] = { (float)qmc5883.raw.x, (float)qmc5883.raw.y, (float)qmc5883.raw.z };
    for(uint8_t k = 0; k < 3; k++){
        if(s[k] < qmc5883.calib.minV[k]) qmc5883.calib.minV[k] = s[k];
        if(s[k] > qmc5883.calib.maxV[k]) qmc5883.calib.maxV[k] = s[k];
    }
}

void QMC5883_CalibFinish(void){
    for(uint8_t k = 0; k < 3; k++){
        qmc5883.calib.offset[k] = (qmc5883.calib.minV[k] + qmc5883.calib.maxV[k]) * 0.5f;
    }
    qmc5883.calib.calibrated = 1;
}

uint8_t QMC5883_DataReady(void){
    uint8_t status = 0;
    if(QMC5883_ReadRegs(QMC5883_REG_STATUS, &status, 1) != QMC5883_OK) return 0;
    return (status & QMC5883_STATUS_DRDY) ? 1 : 0;
}
