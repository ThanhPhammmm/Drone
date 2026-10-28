#ifndef INC_SENSORDRIVER_VL53L1X_VL53L1X_DATA_TYPES_H_
#define INC_SENSORDRIVER_VL53L1X_VL53L1X_DATA_TYPES_H_

#include <stdint.h>

typedef enum{
    VL53L1X_OK = 0,
    VL53L1X_ERROR,
    VL53L1X_TIMEOUT,
    VL53L1X_INVALID_CHIP,
    VL53L1X_CONFIG_ERROR,
} VL53L1X_Status_t;

typedef enum{
    VL53L1X_DISTANCE_SHORT = 1,     /* up to ~1.3 m, better in sunlight */
    VL53L1X_DISTANCE_LONG  = 2,     /* up to ~4 m indoors */
} VL53L1X_DistanceMode_t;

/* range status, as ST's ultra lite driver reports it */
typedef enum{
    VL53L1X_RANGE_VALID          = 0,
    VL53L1X_RANGE_SIGMA_FAIL     = 1,   /* too noisy */
    VL53L1X_RANGE_SIGNAL_FAIL    = 2,   /* too little light came back */
    VL53L1X_RANGE_MIN_RANGE_FAIL = 3,   /* too close */
    VL53L1X_RANGE_OUT_OF_BOUNDS  = 4,   /* phase out of range (target too far) */
    VL53L1X_RANGE_HARDWARE_FAIL  = 5,
    VL53L1X_RANGE_WRAP_FAIL      = 7,   /* wrapped around: target beyond the mode's range */
    VL53L1X_RANGE_NO_UPDATE      = 255,
} VL53L1X_RangeStatus_t;

typedef struct{
    uint16_t distance_mm;
    uint8_t  rangeStatus;           /* VL53L1X_RangeStatus_t */
    uint16_t signalPerSpad;         /* kcps per SPAD */
    uint16_t ambient;               /* kcps */
    uint8_t  numSpads;
} VL53L1X_Result_t;

#endif /* INC_SENSORDRIVER_VL53L1X_VL53L1X_DATA_TYPES_H_ */
