#ifndef INC_FILTER_MAHONY_MAHONY_H_
#define INC_FILTER_MAHONY_MAHONY_H_

#include "bmi088_data_types.h"

typedef struct{
    float kp;
    float ki;

    /* quaternion */
    float q[4];

    /* estimated gyro bias */
    float bias[3];

} Mahony_t;

void Mahony_Init(Mahony_t *mahony, float kp, float ki);
void Mahony_Update(Mahony_t *mahony, BMI088_Data_t *imu);
void Mahony_GetQuaternion(const Mahony_t *mahony, float q[4]);
void Mahony_GetEuler(const Mahony_t *mahony, float *roll, float *pitch, float *yaw);

#endif /* INC_FILTER_MAHONY_MAHONY_H_ */
