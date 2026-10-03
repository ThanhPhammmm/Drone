#ifndef INC_SENSORDRIVER_QMC5883_QMC5883_H_
#define INC_SENSORDRIVER_QMC5883_QMC5883_H_

#include "qmc5883_data_types.h"
#include "qmc5883_reg.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#define QMC5883_CALIB_MIN_SPAN		(0.25f * QMC5883_LSB_PER_GAUSS_8G)

typedef struct{
    QMC5883_Raw_t	raw;
    QMC5883_Field_t field;
    QMC5883_Calib_t calib;
} QMC5883_Handle_t;

extern QMC5883_Handle_t qmc5883;

QMC5883_Status_t QMC5883_Init(void);
uint8_t	QMC5883_DataReady(void);
QMC5883_Status_t QMC5883_Read(void);
void QMC5883_CalibReset(void);
void QMC5883_CalibAccumulate(void);
QMC5883_Status_t QMC5883_CalibFinish(void);
void QMC5883_SetOffsets(float x, float y, float z);

#endif /* INC_SENSORDRIVER_QMC5883_QMC5883_H_ */
