#ifndef INC_SENSORDRIVER_PMW3901_PMW3901_REG_H_
#define INC_SENSORDRIVER_PMW3901_PMW3901_REG_H_

#define PMW3901_REG_PRODUCT_ID          0x00
#define PMW3901_REG_REVISION_ID         0x01
#define PMW3901_REG_MOTION              0x02
#define PMW3901_REG_DELTA_X_L           0x03
#define PMW3901_REG_DELTA_X_H           0x04
#define PMW3901_REG_DELTA_Y_L           0x05
#define PMW3901_REG_DELTA_Y_H           0x06
#define PMW3901_REG_SQUAL               0x07
#define PMW3901_REG_MOTION_BURST        0x16
#define PMW3901_REG_POWER_UP_RESET      0x3A
#define PMW3901_REG_INVERSE_PRODUCT_ID  0x5F

#define PMW3901_PRODUCT_ID_VALUE        0x49
#define PMW3901_INVERSE_ID_VALUE        0xB6
#define PMW3901_POWER_UP_RESET_VALUE    0x5A

#define PMW3901_SPI_WRITE               0x80    /* address bit 7: 1 = write, 0 = read */
#define PMW3901_MOTION_MOT              0x80    /* MOTION bit 7: moved since the last read */

/* motion burst: Motion, Observation, DeltaX L/H, DeltaY L/H, SQUAL,
 * RawData_Sum, Max_RawData, Min_RawData, Shutter_Upper, Shutter_Lower */
#define PMW3901_BURST_LEN               12

#endif /* INC_SENSORDRIVER_PMW3901_PMW3901_REG_H_ */
