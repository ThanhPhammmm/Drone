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

#define TLM_MAGIC               0x5A
#define TLM_FRAME_ATTITUDE      0
#define TLM_FRAME_OUTPUT        1

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

#endif /* INC_RC_RC_PROTOCOL_H_ */