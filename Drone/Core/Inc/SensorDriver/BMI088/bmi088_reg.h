#ifndef INC_SENSORDRIVER_BMI088_BMI088_REG_H_
#define INC_SENSORDRIVER_BMI088_BMI088_REG_H_

/*=========================================================
 *
 * CHIP ID
 *
 *========================================================*/

#define BMI088_ACC_CHIP_ID                 0x1E
#define BMI088_GYRO_CHIP_ID                0x0F

#define BMI088_ACC_CHIP_ID_REG             0x00
#define BMI088_GYRO_CHIP_ID_REG            0x00

/*=========================================================
 *
 * ACCELEROMETER
 *
 *========================================================*/

#define BMI088_ACC_ERR_REG                 0x02
#define BMI088_ACC_STATUS_REG              0x03

#define BMI088_ACC_X_LSB                   0x12
#define BMI088_ACC_X_MSB                   0x13
#define BMI088_ACC_Y_LSB                   0x14
#define BMI088_ACC_Y_MSB                   0x15
#define BMI088_ACC_Z_LSB                   0x16
#define BMI088_ACC_Z_MSB                   0x17

#define BMI088_ACC_TEMP_MSB                0x22
#define BMI088_ACC_TEMP_LSB                0x23

#define BMI088_ACC_CONF                    0x40
#define BMI088_ACC_RANGE                   0x41

#define BMI088_ACC_INT1_IO_CTRL            0x53
#define BMI088_ACC_INT2_IO_CTRL            0x54

#define BMI088_ACC_INT_MAP_DATA            0x58

#define BMI088_ACC_SOFTRESET               0x7E

#define BMI088_ACC_PWR_CONF                0x7C
#define BMI088_ACC_PWR_CTRL                0x7D

/*=========================================================
 *
 * GYROSCOPE
 *
 *========================================================*/

#define BMI088_GYRO_RATE_X_LSB             0x02
#define BMI088_GYRO_RATE_X_MSB             0x03
#define BMI088_GYRO_RATE_Y_LSB             0x04
#define BMI088_GYRO_RATE_Y_MSB             0x05
#define BMI088_GYRO_RATE_Z_LSB             0x06
#define BMI088_GYRO_RATE_Z_MSB             0x07

#define BMI088_GYRO_RANGE                  0x0F
#define BMI088_GYRO_BANDWIDTH              0x10
#define BMI088_GYRO_LPM1                   0x11

#define BMI088_GYRO_SOFTRESET              0x14

#define BMI088_GYRO_INT_CTRL               0x15
#define BMI088_GYRO_INT3_INT4_IO_CONF      0x16
#define BMI088_GYRO_INT3_INT4_IO_MAP       0x18

/*=========================================================
 *
 * RESET VALUE
 *
 *========================================================*/

#define BMI088_SOFT_RESET_CMD              0xB6

/*=========================================================
 *
 * ACC RANGE
 *
 *========================================================*/

#define BMI088_ACC_RANGE_3G                0x00
#define BMI088_ACC_RANGE_6G                0x01
#define BMI088_ACC_RANGE_12G               0x02
#define BMI088_ACC_RANGE_24G               0x03

/*=========================================================
 *
 * GYRO RANGE
 *
 *========================================================*/

#define BMI088_GYRO_RANGE_2000             0x00
#define BMI088_GYRO_RANGE_1000             0x01
#define BMI088_GYRO_RANGE_500              0x02
#define BMI088_GYRO_RANGE_250              0x03
#define BMI088_GYRO_RANGE_125              0x04

/*=========================================================
 *
 * ACC ODR
 *
 *========================================================*/

#define BMI088_ACC_ODR_12                  0x05
#define BMI088_ACC_ODR_25                  0x06
#define BMI088_ACC_ODR_50                  0x07
#define BMI088_ACC_ODR_100                 0x08
#define BMI088_ACC_ODR_200                 0x09
#define BMI088_ACC_ODR_400                 0x0A
#define BMI088_ACC_ODR_800                 0x0B
#define BMI088_ACC_ODR_1600                0x0C

/*=========================================================
 *
 * GYRO ODR
 *
 *========================================================*/

#define BMI088_GYRO_ODR_2000_BW_532        0x00
#define BMI088_GYRO_ODR_2000_BW_230        0x01
#define BMI088_GYRO_ODR_1000_BW_116        0x02
#define BMI088_GYRO_ODR_400_BW_47          0x03
#define BMI088_GYRO_ODR_200_BW_23          0x04
#define BMI088_GYRO_ODR_100_BW_12          0x05
#define BMI088_GYRO_ODR_200_BW_64          0x06
#define BMI088_GYRO_ODR_100_BW_32          0x07

#endif /* INC_SENSORDRIVER_BMI088_BMI088_REG_H_ */
