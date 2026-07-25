#ifndef INC_SENSORDRIVER_QMC5883_QMC5883_REG_H_
#define INC_SENSORDRIVER_QMC5883_QMC5883_REG_H_

#define QMC5883_I2C_ADDR             (0x2C << 1)   /* 7-bit 0x2C, HAL wants 8-bit form */

#define QMC5883_REG_CHIP_ID          0x00
#define QMC5883_REG_DATA_X_LSB       0x01
#define QMC5883_REG_DATA_X_MSB       0x02
#define QMC5883_REG_DATA_Y_LSB       0x03
#define QMC5883_REG_DATA_Y_MSB       0x04
#define QMC5883_REG_DATA_Z_LSB       0x05
#define QMC5883_REG_DATA_Z_MSB       0x06
#define QMC5883_REG_STATUS           0x09
#define QMC5883_REG_CTRL1            0x0A
#define QMC5883_REG_CTRL2            0x0B
#define QMC5883_REG_SIGN             0x29   /* not in the register table, but every
                                                setup example writes 0x06 here first */

#define QMC5883_CHIP_ID_VALUE        0x80
#define QMC5883_STATUS_DRDY          0x01
#define QMC5883_STATUS_OVFL          0x02

/* Control Register 1 (0x0A): [7:6]=OSR2 [5:4]=OSR1 [3:2]=ODR [1:0]=MODE */
#define QMC5883_MODE_SUSPEND         0x00
#define QMC5883_MODE_NORMAL          0x01
#define QMC5883_MODE_SINGLE          0x02
#define QMC5883_MODE_CONTINUOUS      0x03

#define QMC5883_ODR_10HZ             (0x00 << 2)
#define QMC5883_ODR_50HZ             (0x01 << 2)
#define QMC5883_ODR_100HZ            (0x02 << 2)
#define QMC5883_ODR_200HZ            (0x03 << 2)

#define QMC5883_OSR1_8               (0x00 << 4)
#define QMC5883_OSR1_4               (0x01 << 4)
#define QMC5883_OSR1_2               (0x02 << 4)
#define QMC5883_OSR1_1               (0x03 << 4)

#define QMC5883_OSR2_1               (0x00 << 6)
#define QMC5883_OSR2_2               (0x01 << 6)
#define QMC5883_OSR2_4               (0x02 << 6)
#define QMC5883_OSR2_8               (0x03 << 6)

/* Control Register 2 (0x0B): [7]=SOFT_RST [6]=SELF_TEST [3:2]=RNG [1:0]=SET/RESET MODE */
#define QMC5883_CTRL2_SOFT_RST       0x80
#define QMC5883_CTRL2_SELF_TEST      0x40

#define QMC5883_RNG_30G              (0x00 << 2)
#define QMC5883_RNG_12G              (0x01 << 2)
#define QMC5883_RNG_8G               (0x02 << 2)
#define QMC5883_RNG_2G               (0x03 << 2)

#define QMC5883_SETRESET_ON          0x00
#define QMC5883_SETRESET_SETONLY     0x01
#define QMC5883_SETRESET_OFF         0x02

#define QMC5883_LSB_PER_GAUSS_8G     3750.0f   /* datasheet: Field Range=8G -> 3750 LSB/G */

#endif /* INC_SENSORDRIVER_QMC5883_QMC5883_REG_H_ */
