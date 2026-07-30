#ifndef INC_SENSORDRIVER_QMC5883_QMC5883_H_
#define INC_SENSORDRIVER_QMC5883_QMC5883_H_

#include "qmc5883_data_types.h"

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
void QMC5883_CalibFinish(void);

#endif /* INC_SENSORDRIVER_QMC5883_QMC5883_H_ */
