#ifndef INC_RC_RC_PROTOCOL_H_
#define INC_RC_RC_PROTOCOL_H_

#include <stdint.h>

/* NOTE: this file must stay byte-for-byte identical to
 * Drone/Core/Inc/RC/rc_protocol.h on the drone side. If the drone-side
 * protocol changes, copy it back over this file rather than editing the
 * two independently. */


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

/* Telemetry: one frame per ACK (32 bytes at most), the four in turn */
#define TLM_MAGIC               0x5A
#define TLM_FRAME_ATTITUDE      0       /* [IMU] [ATT] */
#define TLM_FRAME_OUTPUT        1       /* [MOT] [ALT] */
#define TLM_FRAME_SENSOR        2       /* [POS] [TOF] [FLOW] [MISC] */
#define TLM_FRAME_GPS           3       /* [GPS] */

/* state byte */
#define TLM_STATE_ARM_MASK      0x07    /* arm_state_t: 0 DISARMED .. 6 FAILSAFE */
#define TLM_STATE_MODE_SHIFT    3
#define TLM_STATE_MODE_MASK     0x18    /* flight mode in force: 0 ANGLE, 1 ALT_HOLD, 2 POS_HOLD (3 free) */
/* calibration flags = state >> 5: 1 IMU, 2 baro, 4 compass (7 = all calibrated) */
#define TLM_STATE_IMU_CAL       0x20    /* BMI088 calibrated */
#define TLM_STATE_BARO_CAL      0x40    /* BMP388 ground reference taken, baro data flowing */
#define TLM_STATE_MAG_CAL       0x80    /* QMC5883 calibrated (not needed to fly) */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_ATTITUDE */
    uint8_t  rcSeq;             /* seq of the last RC packet the drone received */
    uint8_t  state;
    int16_t  gyro[3];           /* body rates x/y/z (roll/pitch/yaw), 0.1 deg/s */
    int16_t  accel[3];          /* accelerometer x/y/z (front/right/down), 0.01 m/s^2: still and level, z = -9.81 */
    int16_t  roll, pitch, yaw;  /* estimate, 0.01 deg */
    int16_t  rollSp, pitchSp;   /* angle flown now (sticks, or the position controller), 0.01 deg */
    int16_t  yawRateSp;         /* yaw rate setpoint, 0.1 deg/s */
    uint8_t  tiltDeg;           /* tilt from level, deg */
} TLM_Attitude_t;               /* 29 bytes */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_OUTPUT */
    uint8_t  rcSeq;
    uint8_t  state;
    uint16_t motor[4];          /* pulse, us: M1 FR, M2 RR, M3 RL, M4 FL */
    uint16_t thrust;            /* collective 0..1, 1e-4 */
    uint16_t hoverThrust;       /* hover thrust measured in flight, 1e-4; 0 = not known yet */
    int16_t  altitude;          /* estimate, cm */
    int16_t  altitudeSp;        /* height it holds (ALT_HOLD, POS_HOLD), cm */
    int16_t  vz;                /* vertical speed, cm/s, +up */
    int16_t  climbSp;           /* climb rate the stick asks, cm/s */
    int16_t  accelUp;           /* vertical acceleration, cm/s^2 */
    int16_t  accelBias;         /* altitude estimator's accel bias, cm/s^2 */
    int16_t  aboveFloor;        /* height above the floor, cm */
} TLM_Output_t;                 /* 30 bytes */

/* valid byte (sensor frame) */
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
    int16_t  x, y;              /* position estimate, cm, level frame: x = the nose at power-up, y = its right */
    int16_t  xSp, ySp;          /* position it holds (POS_HOLD), cm */
    int16_t  vx, vy;            /* velocity estimate, cm/s, level frame */
    uint16_t rangeMm;           /* ToF distance along the sensor axis, mm */
    int16_t  rangeHeight;       /* ToF height, tilt corrected, cm */
    uint8_t  rangeStatus;       /* ToF status of the last sample: 0 ok, 1 sigma, 2 signal, 3 too close,
                                   4 too far, 7 wrap, 255 no sample yet */
    int16_t  flowX, flowY;      /* PMW3901 counts in the last sample, sensor axes */
    uint8_t  squal;             /* PMW3901 surface quality */
    uint8_t  vibration;         /* | |raw accel| - 1 g | at the sample, 0.1 m/s^2 */
    uint16_t rcLost;            /* RC packets missed so far (seq gaps) */
} TLM_Sensor_t;                 /* 30 bytes */

/* status byte (GPS frame) */
#define TLM_GPS_FIX_MASK        0x07    /* fix type: 0 none, 1 dead reckoning, 2 2D, 3 3D, 4 GNSS + DR, 5 time only */
#define TLM_GPS_FIX_OK          0x08    /* the receiver trusts its fix */
#define TLM_GPS_FOUND           0x10    /* the receiver answers on USART1 */
#define TLM_GPS_ALIGNED         0x20    /* heading tied to north: the GPS can hold the position */
#define TLM_GPS_SOURCE_SHIFT    6
#define TLM_GPS_SOURCE_MASK     0xC0    /* what measures the position now: 0 none, 1 flow, 2 GPS */

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  type;              /* TLM_FRAME_GPS */
    uint8_t  rcSeq;
    uint8_t  state;
    uint8_t  status;            /* TLM_GPS_* */
    uint8_t  numSV;             /* satellites used */
    int32_t  lat;               /* 1e-7 deg */
    int32_t  lon;               /* 1e-7 deg */
    int16_t  altMsl;            /* 0.1 m above mean sea level */
    uint16_t hAcc;              /* cm, horizontal accuracy the receiver reports */
    int16_t  velN, velE;        /* cm/s */
    uint16_t sAcc;              /* cm/s, speed accuracy the receiver reports */
    int16_t  yawOffset;         /* 0.01 deg, from north to the level frame's x */
    int16_t  gpsX, gpsY;        /* cm, the GPS fix in the level frame (0 until aligned): compare with x, y */
} TLM_Gps_t;                    /* 30 bytes */

#endif /* INC_RC_RC_PROTOCOL_H_ */
