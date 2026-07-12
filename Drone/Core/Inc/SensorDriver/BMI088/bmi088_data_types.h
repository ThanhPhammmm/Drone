#ifndef INC_SENSORDRIVER_BMI088_BMI088_DATA_TYPES_H_
#define INC_SENSORDRIVER_BMI088_BMI088_DATA_TYPES_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;

} BMI088_AccelRaw_t;

typedef struct{
    int16_t x;
    int16_t y;
    int16_t z;

} BMI088_GyroRaw_t;

typedef struct{
    float x;
    float y;
    float z;

} BMI088_Accel_t;

typedef struct{
    float x;
    float y;
    float z;

} BMI088_Gyro_t;

typedef struct{
    BMI088_AccelRaw_t accel_raw;
    BMI088_GyroRaw_t gyro_raw;

    BMI088_Accel_t accel;
    BMI088_Gyro_t gyro;

    uint32_t timestamp;

} BMI088_Data_t;

typedef enum{
    BMI088_OK = 0,

    BMI088_ERROR,

    BMI088_TIMEOUT,

    BMI088_INVALID_CHIP,

    BMI088_DMA_BUSY,

} BMI088_Status_t;


#endif /* INC_SENSORDRIVER_BMI088_BMI088_DATA_TYPES_H_ */
