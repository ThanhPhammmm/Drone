#ifndef INC_RC_RC_PROTOCOL_H_
#define INC_RC_RC_PROTOCOL_H_

#include <stdint.h>


#define RC_PACKET_MAGIC     0xA5

#define RC_CHANNEL_MAX      1000        /* sticks:   -1000..+1000 */
#define RC_THROTTLE_MAX     1000        /* throttle:     0..1000  */

typedef enum{
	RC_MODE_ANGLE = 0,
	RC_MODE_ALT_HOLD,
	RC_MODE_POS_HOLD,
} RC_Mode_t;

#define RC_FLAG_ARM         0x01

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  seq;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
    uint16_t throttle;
    uint8_t  mode;
    uint8_t  flags;
} RC_Packet_t;              /* 12 bytes */

#define TLM_MAGIC               0x5A
#define TLM_FRAME_ATTITUDE      0
#define TLM_FRAME_OUTPUT        1
#define TLM_FRAME_SENSOR        2

/* state byte */
#define TLM_STATE_ARM_MASK      0x07    /* arm_state_t: 0 DISARMED .. 6 FAILSAFE */
#define TLM_STATE_MODE_SHIFT    3
#define TLM_STATE_MODE_MASK     0x18    /* flight mode in force: 0 ANGLE, 1 ALT_HOLD, 2 POS_HOLD */
/* calibration flags = state >> 5: 1 IMU, 2 baro, 4 compass (7 = all calibrated) */
#define TLM_STATE_IMU_CAL       0x20    /* BMI088 calibrated */
#define TLM_STATE_BARO_CAL      0x40    /* BMP388 ground reference taken, baro data flowing */
#define TLM_STATE_MAG_CAL       0x80    /* QMC5883 calibrated (not needed to fly ANGLE / ALT_HOLD) */

/* sat byte: what the mixer had to cut on the last rate-loop step */
#define TLM_SAT_ROLL_PITCH      0x01
#define TLM_SAT_YAW             0x02
#define TLM_SAT_THR_HIGH        0x04
#define TLM_SAT_THR_LOW         0x08
#define TLM_SAT_TUMBLE          0x10    /* not the mixer: the last disarm was the tumble cut-off */
#define TLM_SAT_ALT_VALID       0x20    /* not the mixer: altitude estimate valid (output frame only) */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_ATTITUDE */
    uint8_t  rcSeq;             /* seq of the last RC packet the drone received */
    uint8_t  state;
    int16_t  roll, pitch, yaw;  /* estimate, 0.01 deg */
    int16_t  rollSp, pitchSp;   /* angle setpoint, 0.01 deg */
    int16_t  rate[3];           /* gyro roll/pitch/yaw rate, 0.1 deg/s */
    int16_t  rateSp[3];         /* rate setpoint, 0.1 deg/s */
    int16_t  torque[3];         /* rate PID output roll/pitch/yaw, 1e-4 */
} TLM_Attitude_t;               /* 32 bytes */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_OUTPUT */
    uint8_t  rcSeq;
    uint8_t  state;
    uint16_t motor[4];          /* pulse, us: M1 FR, M2 RR, M3 RL, M4 FL */
    uint16_t thrust;            /* collective 0..1, 1e-4 */
    int16_t  altitude;          /* estimate, cm */
    int16_t  baroAltitude;      /* raw baro, cm */
    int16_t  vz;                /* vertical speed, cm/s, +up */
    int16_t  climbSp;           /* ALT_HOLD climb-rate setpoint, cm/s */
    int16_t  accelUp;           /* vertical acceleration, cm/s^2 */
    uint16_t rateDtMaxUs;       /* longest rate-loop step since the previous frame */
    uint16_t rcLost;            /* RC packets missed so far (seq gaps) */
    uint16_t hoverThrust;       /* hover thrust measured in flight, 1e-4; 0 = not known yet */
    uint8_t  sat;               /* TLM_SAT_* */
    uint8_t  vibration;         /* | |raw accel| - 1 g | at the sample, 0.1 m/s^2 */
} TLM_Output_t;                 /* 32 bytes */

/* valid byte (sensor frame): what can be used right now */
#define TLM_VALID_BARO          0x01    /* baro samples reach the altitude estimator */
#define TLM_VALID_RANGE         0x02    /* ToF: last sample valid (status 0) and fresh */
#define TLM_VALID_FLOW          0x04    /* flow: last sample good (squal) and fresh */
#define TLM_VALID_RANGE_INIT    0x08    /* VL53L1X initialized OK */
#define TLM_VALID_FLOW_INIT     0x10    /* PMW3901 initialized OK */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_SENSOR */
    uint8_t  rcSeq;
    uint8_t  state;
    uint8_t  valid;             /* TLM_VALID_* */
    uint8_t  rangeStatus;       /* ToF status of the last sample: 0 ok, 1 sigma, 2 signal, 3 too close,
                                   4 too far, 7 wrap, 255 no sample yet */
    uint16_t rangeMm;           /* ToF distance along the sensor axis, mm */
    int16_t  rangeHeight;       /* ToF height, tilt corrected, cm */
    int16_t  accelBias;         /* altitude estimator's accel bias, cm/s^2 */
    int16_t  flowX, flowY;      /* PMW3901 counts in the last sample, sensor axes */
    uint8_t  flowDtMs;          /* time the last flow sample covers, ms */
    uint8_t  squal;             /* PMW3901 surface quality */
    uint8_t  rangeAgeMs;        /* time since the last sample, ms; 255 = none or older */
    uint8_t  flowAgeMs;
    uint8_t  baroAgeMs;
    uint8_t  tiltDeg;           /* tilt from level: ToF height and flow get worse past ~30 deg */
} TLM_Sensor_t;                 /* 22 bytes */

#endif /* INC_RC_RC_PROTOCOL_H_ */