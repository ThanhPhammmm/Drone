
#ifndef INC_SENSORDRIVER_BMP388_BMP388_REG_H_
#define INC_SENSORDRIVER_BMP388_BMP388_REG_H_

#define BMP388_I2C_ADDR_PRIMARY      (0x76 << 1)   /* SDO -> GND */
#define BMP388_I2C_ADDR_SECONDARY    (0x77 << 1)   /* SDO -> VDD (Adafruit default) */

#define BMP388_REG_CHIP_ID           0x00
#define BMP388_REG_ERR               0x02
#define BMP388_REG_STATUS            0x03
#define BMP388_REG_DATA_0            0x04
#define BMP388_REG_SENSORTIME        0x0C
#define BMP388_REG_EVENT             0x10
#define BMP388_REG_INT_STATUS        0x11
#define BMP388_REG_PWR_CTRL          0x1B
#define BMP388_REG_OSR               0x1C
#define BMP388_REG_ODR               0x1D
#define BMP388_REG_CONFIG            0x1F
#define BMP388_REG_CALIB_DATA        0x31
#define BMP388_REG_CMD               0x7E

#define BMP388_CHIP_ID_VALUE         0x50
#define BMP390_CHIP_ID_VALUE         0x60
#define BMP388_CALIB_LEN             21

#define BMP388_CMD_SOFT_RESET        0xB6

/* ERR_REG (0x02) */
#define BMP388_ERR_FATAL             0x01
#define BMP388_ERR_CMD               0x02
#define BMP388_ERR_CONF              0x04

#define BMP388_STATUS_CMD_RDY        0x10
#define BMP388_STATUS_DRDY_PRESS     0x20
#define BMP388_STATUS_DRDY_TEMP      0x40

/* PWR_CTRL (0x1B): [1:0]=enables [5:4]=mode */
#define BMP388_PWR_PRESS_EN          0x01
#define BMP388_PWR_TEMP_EN           0x02
#define BMP388_PWR_MODE_SLEEP        (0x00 << 4)
#define BMP388_PWR_MODE_FORCED       (0x01 << 4)
#define BMP388_PWR_MODE_NORMAL       (0x03 << 4)

/* OSR (0x1C): [2:0]=osr_p [5:3]=osr_t */
#define BMP388_OSR_P_1               0x00
#define BMP388_OSR_P_2               0x01
#define BMP388_OSR_P_4               0x02
#define BMP388_OSR_P_8               0x03
#define BMP388_OSR_P_16              0x04
#define BMP388_OSR_P_32              0x05

#define BMP388_OSR_T_1               (0x00 << 3)
#define BMP388_OSR_T_2               (0x01 << 3)
#define BMP388_OSR_T_4               (0x02 << 3)
#define BMP388_OSR_T_8               (0x03 << 3)

/* ODR (0x1D): [4:0]=odr_sel, each step halves the rate from 200 Hz */
#define BMP388_ODR_200HZ             0x00
#define BMP388_ODR_100HZ             0x01
#define BMP388_ODR_50HZ              0x02
#define BMP388_ODR_25HZ              0x03
#define BMP388_ODR_12P5HZ            0x04

/* CONFIG (0x1F): [3:1]=iir_filter */
#define BMP388_IIR_BYPASS            (0x00 << 1)
#define BMP388_IIR_COEF_1            (0x01 << 1)
#define BMP388_IIR_COEF_3            (0x02 << 1)
#define BMP388_IIR_COEF_7            (0x03 << 1)
#define BMP388_IIR_COEF_15           (0x04 << 1)
#define BMP388_IIR_COEF_31           (0x05 << 1)
#define BMP388_IIR_COEF_63           (0x06 << 1)
#define BMP388_IIR_COEF_127          (0x07 << 1)

#endif /* INC_SENSORDRIVER_BMP388_BMP388_REG_H_ */