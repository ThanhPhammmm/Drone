#ifndef INC_RC_TELEMETRY_H_
#define INC_RC_TELEMETRY_H_

#include <stdint.h>

typedef struct{
    /* link, measured here */
    uint8_t  linkQuality;       /* % of the last 50 RC packets the drone ACKed */
    uint32_t frames;            /* telemetry frames received */
    uint32_t lastFrameTick;

    /* drone state */
    uint8_t  armState;          /* 0 DISARMED 1 ARMING 2 ARMED_GROUND 3 TAKEOFF 4 AIRBORNE 5 LANDING 6 FAILSAFE */
    uint8_t  mode;              /* 0 ANGLE 1 ALT_HOLD 2 POS_HOLD */
    uint8_t  imuCalibrated;
    uint8_t  baroCalibrated;
    uint8_t  magCalibrated;
    uint8_t  altitudeValid;

    /* attitude loop */
    float    roll, pitch, yaw;  /* deg */
    float    rollSp, pitchSp;   /* deg */
    float    rate[3];           /* deg/s, roll pitch yaw */
    float    rateSp[3];         /* deg/s */
    float    torque[3];         /* rate PID output */

    /* outputs */
    uint16_t motor[4];          /* us: M1 FR, M2 RR, M3 RL, M4 FL */
    float    thrust;            /* 0..1 */
    float    hoverThrust;       /* 0..1, measured in flight; 0 = not known yet */
    uint8_t  saturation;        /* 1 roll/pitch cut, 2 yaw cut, 4 throttle at max, 8 throttle at min */

    /* altitude */
    float    altitude;          /* m */
    float    baroAltitude;      /* m */
    float    verticalSpeed;     /* m/s */
    float    climbSp;           /* m/s */
    float    accelUp;           /* m/s^2 */
    float    accelBias;         /* m/s^2, altitude estimator's accel bias */
    uint8_t  baroAgeMs;         /* since the last baro sample, 255 = none */

    /* range and flow sensors (sensor frame) */
    uint8_t  sensorValid;       /* TLM_VALID_ */
    uint8_t  rangeReady;        /* VL53L1X initialized */
    uint8_t  flowReady;         /* PMW3901 initialized */
    float    range;             /* m, ToF along the sensor axis */
    float    rangeHeight;       /* m, ToF tilt corrected */
    uint8_t  rangeStatus;       /* 0 ok, 1 sigma, 2 signal, 3 too close, 4 too far, 7 wrap, 255 none */
    uint8_t  rangeAgeMs;
    int16_t  flowX, flowY;      /* counts in the last sample */
    uint8_t  flowDtMs;
    uint8_t  squal;
    uint8_t  flowAgeMs;
    uint8_t  tiltDeg;

    /* health */
    float    vibration;         /* m/s^2 */
    uint16_t rateDtMaxUs;
    uint16_t rcLost;
} Telemetry_t;

extern Telemetry_t telemetry;

void Telemetry_Decode(const uint8_t *buf, uint8_t len);
void Telemetry_LinkResult(uint8_t acked);
void TelemetryPrintTask(void *argument);

#endif /* INC_RC_TELEMETRY_H_ */