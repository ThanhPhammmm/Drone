#include "telemetry.h"
#include "rc_protocol.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include "rc_tx_task.h"
#include "joystick.h"

#define TLM_PRINT_PERIOD_MS     200U         /* poll; one line per new output frame (25 Hz) */
#define TLM_STALE_MS            500U
#define TLM_HEADER_EVERY        250U        /* lines (~10 s), for a terminal opened late */
#define TLM_DMA_TIMEOUT_MS      200U        /* a full 768-byte line takes ~67 ms at 115200 baud */

extern UART_HandleTypeDef huart1;

Telemetry_t telemetry;

static TLM_Attitude_t rawAtt;
static TLM_Output_t   rawOut;
static TLM_Sensor_t   rawSen = {             /* "no sample" until the first sensor frame */
    .rangeStatus = 255, .rangeAgeMs = 255, .flowAgeMs = 255, .baroAgeMs = 255,
};
static uint32_t       outCount;
static uint8_t        linkPct;
static uint64_t       linkHistory;          /* 1 bit per packet, newest in bit 0 */

void Telemetry_LinkResult(uint8_t acked){
    linkHistory = (linkHistory << 1) | (acked ? 1U : 0U);
    uint8_t n = (uint8_t)__builtin_popcountll(linkHistory & ((1ULL << 50) - 1ULL));
    linkPct = (uint8_t)(n * 2U);
    telemetry.linkQuality = linkPct;
}

static void Telemetry_DecodeState(uint8_t s){
    telemetry.armState          = s & TLM_STATE_ARM_MASK;
    telemetry.mode              = (s & TLM_STATE_MODE_MASK) >> TLM_STATE_MODE_SHIFT;
    telemetry.imuCalibrated     = (s & TLM_STATE_IMU_CAL)     ? 1 : 0;
    telemetry.baroCalibrated    = (s & TLM_STATE_BARO_CAL)    ? 1 : 0;
    telemetry.magCalibrated     = (s & TLM_STATE_MAG_CAL)     ? 1 : 0;
}

void Telemetry_Decode(const uint8_t *buf, uint8_t len){
    if(len < 2 || buf[0] != TLM_MAGIC) return;

    if(buf[1] == TLM_FRAME_ATTITUDE && len == sizeof(TLM_Attitude_t)){
        TLM_Attitude_t f;
        memcpy(&f, buf, sizeof(f));

        taskENTER_CRITICAL();
        rawAtt = f;
        taskEXIT_CRITICAL();

        Telemetry_DecodeState(f.state);
        telemetry.roll    = f.roll    * 0.01f;
        telemetry.pitch   = f.pitch   * 0.01f;
        telemetry.yaw     = f.yaw     * 0.01f;
        telemetry.rollSp  = f.rollSp  * 0.01f;
        telemetry.pitchSp = f.pitchSp * 0.01f;
        for(uint8_t i = 0; i < 3; i++){
            telemetry.rate[i]   = f.rate[i]   * 0.1f;
            telemetry.rateSp[i] = f.rateSp[i] * 0.1f;
            telemetry.torque[i] = f.torque[i] * 0.0001f;
        }
    }
    else if(buf[1] == TLM_FRAME_OUTPUT && len == sizeof(TLM_Output_t)){
        TLM_Output_t f;
        memcpy(&f, buf, sizeof(f));

        taskENTER_CRITICAL();
        rawOut = f;
        outCount++;
        taskEXIT_CRITICAL();

        Telemetry_DecodeState(f.state);
        for(uint8_t i = 0; i < 4; i++) telemetry.motor[i] = f.motor[i];
        telemetry.thrust        = f.thrust       * 0.0001f;
        telemetry.hoverThrust   = f.hoverThrust  * 0.0001f;
        telemetry.saturation    = f.sat;
        telemetry.altitudeValid = (f.sat & TLM_SAT_ALT_VALID) ? 1 : 0;
        telemetry.altitude      = f.altitude     * 0.01f;
        telemetry.baroAltitude  = f.baroAltitude * 0.01f;
        telemetry.verticalSpeed = f.vz           * 0.01f;
        telemetry.climbSp       = f.climbSp      * 0.01f;
        telemetry.accelUp       = f.accelUp      * 0.01f;
        telemetry.vibration     = f.vibration    * 0.1f;
        telemetry.rateDtMaxUs   = f.rateDtMaxUs;
        telemetry.rcLost        = f.rcLost;
    }
    else if(buf[1] == TLM_FRAME_SENSOR && len == sizeof(TLM_Sensor_t)){
        TLM_Sensor_t f;
        memcpy(&f, buf, sizeof(f));

        taskENTER_CRITICAL();
        rawSen = f;
        taskEXIT_CRITICAL();

        Telemetry_DecodeState(f.state);
        telemetry.sensorValid   = f.valid;
        telemetry.rangeReady    = (f.valid & TLM_VALID_RANGE_INIT) ? 1 : 0;
        telemetry.flowReady     = (f.valid & TLM_VALID_FLOW_INIT)  ? 1 : 0;
        telemetry.range         = f.rangeMm     * 0.001f;
        telemetry.rangeHeight   = f.rangeHeight * 0.01f;
        telemetry.rangeStatus   = f.rangeStatus;
        telemetry.rangeAgeMs    = f.rangeAgeMs;
        telemetry.accelBias     = f.accelBias   * 0.01f;
        telemetry.flowX         = f.flowX;
        telemetry.flowY         = f.flowY;
        telemetry.flowDtMs      = f.flowDtMs;
        telemetry.squal         = f.squal;
        telemetry.flowAgeMs     = f.flowAgeMs;
        telemetry.baroAgeMs     = f.baroAgeMs;
        telemetry.tiltDeg       = f.tiltDeg;
    }
    else{
        return;
    }
    telemetry.frames++;
    telemetry.lastFrameTick = xTaskGetTickCount();
}

static char *PutU32(char *p, uint32_t v){
    char tmp[10];
    uint8_t n = 0;
    do{ tmp[n++] = (char)('0' + v % 10U); v /= 10U; }while(v);
    while(n) *p++ = tmp[--n];
    return p;
}

static char *PutFixed(char *p, int32_t v, uint8_t decimals){
    static const uint32_t pow10[] = { 1, 10, 100, 1000, 10000 };
    uint32_t a = (v < 0) ? (uint32_t)(-v) : (uint32_t)v;
    if(v < 0) *p++ = '-';
    p = PutU32(p, a / pow10[decimals]);
    if(decimals){
        *p++ = '.';
        uint32_t frac = a % pow10[decimals];
        for(uint32_t d = pow10[decimals] / 10U; d; d /= 10U){
            *p++ = (char)('0' + (frac / d) % 10U);
        }
    }
    return p;
}

static char *PutStr(char *p, const char *s){
    while(*s) *p++ = *s++;
    return p;
}

static TaskHandle_t tlmTask;

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart != &huart1 || tlmTask == NULL) return;
    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(tlmTask, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void TLM_WaitIdle(void){
    if(huart1.gState == HAL_UART_STATE_READY) return;
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(TLM_DMA_TIMEOUT_MS));
    if(huart1.gState != HAL_UART_STATE_READY) HAL_UART_AbortTransmit(&huart1);   /* stuck: start over */
}

static void TLM_Write(const char *s, uint16_t len){
    TLM_WaitIdle();
    ulTaskNotifyTake(pdTRUE, 0);        /* drop a completion nobody waited for */
    HAL_UART_Transmit_DMA(&huart1, (uint8_t *)s, len);
}

#define FIELD_FIXED(v, d)   do{ p = PutFixed(p, (int32_t)(v), (d)); *p++ = ','; }while(0)
#define FIELD_LABELED(label, v, d) \
    do{ p = PutStr(p, label); p = PutFixed(p, (int32_t)(v), (d)); }while(0)

static const char tlmHeader[] =
    "t_ms,link,arm,mode,flags,"
    "roll,pitch,yaw,rollSp,pitchSp,"
    "gx,gy,gz,gxSp,gySp,gzSp,"
    "tqR,tqP,tqY,"
    "m1,m2,m3,m4,thr,hover,sat,"
    "alt,baroAlt,vz,climbSp,accUp,"
    "vib,dtMax,rcLost\r\n";

static const char tlmLegend[] =
    "# drone telemetry, ~17 Hz. angles deg, rates deg/s, motors us, alt m, vz m/s, vib m/s^2, dtMax us\r\n"
    "# arm: 0 DISARMED 1 ARMING 2 ARMED_GROUND 3 TAKEOFF 4 AIRBORNE 5 LANDING 6 FAILSAFE\r\n"
    "# mode: 0 ANGLE 1 ALT_HOLD 2 POS_HOLD | (IMU MAG BARO RANGE FLOW): sensor ready, noXXX = not. IMU/MAG/BARO calibrated, RANGE/FLOW initialized\r\n"    "# YawSp: yaw rate setpoint, deg/s (yaw is flown as a rate) | Hover: hover thrust measured in flight (0 = not known yet, set once AIRBORNE)\r\n"
    "# sat: 1 roll/pitch cut, 2 yaw cut, 4 throttle at max, 8 throttle at min, 16 TUMBLE CUT-OFF (motors stopped) | Valid: altitude estimate usable (1 = ALT_HOLD possible)\r\n"
    "# motors: M1 front-right, M2 rear-right, M3 rear-left, M4 front-left\r\n"
    "# [ALT] BaroOk: baro reaching the estimator, BaroAge ms, ABias m/s^2 estimator accel bias\r\n"
    "# [TOF] Dist m along the sensor, Height m tilt corrected, Status 0 ok 1 sigma 2 signal 3 too close 4 too far 7 wrap 255 none, Age ms (255 = none), Ok: valid and fresh, Tilt deg\r\n"
    "# [FLOW] X/Y counts in the last sample, Dt ms it covers, Squal surface quality (>= 25 usable), Age ms, Ok: good and fresh\r\n"
    "# [RC]: what this remote commands now (RollSp/PitchSp deg, YawSp deg/s); [ATT] RollSp/PitchSp/YawSp: what the drone got\r\n";

/* Stick -> setpoint, the same scaling as the drone's rc_task.c
 * (RC_MAX_TILT_RAD, RC_MAX_YAW_RATE): keep the two in step. */
#define TLM_RC_MAX_TILT_DEG     (0.35f * 57.29578f)     /* full stick = ~20 deg */
#define TLM_RC_MAX_YAW_DPS      (3.0f  * 57.29578f)     /* full stick = ~172 deg/s */

static float TLM_StickNorm(int16_t v){
    float f = (float)v / (float)RC_CHANNEL_MAX;
    if(f >  1.0f) f =  1.0f;
    if(f < -1.0f) f = -1.0f;
    return f;
}

static char *PutStick(char *p){
    RCTx_Status_t st;
    uint8_t have = RCTx_GetStatus(&st);
    if(!have) memset(&st, 0, sizeof(st));

    p = PutStr(p, " | [RC] ");
    FIELD_LABELED("RollSp:",   TLM_StickNorm(st.lastSent.roll)  * TLM_RC_MAX_TILT_DEG * 100.0f, 2);
    FIELD_LABELED(", PitchSp:", TLM_StickNorm(st.lastSent.pitch) * TLM_RC_MAX_TILT_DEG * 100.0f, 2);
    FIELD_LABELED(", YawSp:",   TLM_StickNorm(st.lastSent.yaw)   * TLM_RC_MAX_YAW_DPS  * 10.0f,  1);
    FIELD_LABELED(", ModeSent:", st.lastSent.mode, 0);
    FIELD_LABELED(", ArmSw:", (st.lastSent.flags & RC_FLAG_ARM) ? 1 : 0, 0);
    p = PutStr(p, ", ThrRaw:");
    p = PutU32(p, JOY_ThrottleRaw());
    p = PutStr(p, "/4095, ThrSent:");
    p = PutU32(p, st.lastSent.throttle);
    p = PutStr(p, "/1000");
    return p;
}

static uint16_t Telemetry_FormatLine(char *buf, const TLM_Attitude_t *a, const TLM_Output_t *o,
                                     const TLM_Sensor_t *s, uint8_t link){
    char *p = buf;

    p = PutStr(p, "[SYS] ");
    FIELD_LABELED("Time:", xTaskGetTickCount() * portTICK_PERIOD_MS, 0);
    FIELD_LABELED("ms, Link:", link, 0);
    FIELD_LABELED("%, Arm:", o->state & TLM_STATE_ARM_MASK, 0);
    FIELD_LABELED(", Mode:", (o->state & TLM_STATE_MODE_MASK) >> TLM_STATE_MODE_SHIFT, 0);
    p = PutStr(p, (o->state & TLM_STATE_IMU_CAL) ? " (IMU" : " (noIMU");
    p = PutStr(p, (o->state & TLM_STATE_MAG_CAL) ? " MAG"  : " noMAG");
    p = PutStr(p, ((o->state & TLM_STATE_BARO_CAL)) ? " BARO" : " noBARO");
    p = PutStr(p, (s->valid & TLM_VALID_RANGE_INIT) ? " RANGE" : " noRANGE");
    p = PutStr(p, (s->valid & TLM_VALID_FLOW_INIT)  ? " FLOW)" : " noFLOW)");

    p = PutStr(p, " | [ATT] ");
    FIELD_LABELED("Roll:", a->roll, 2);
    FIELD_LABELED(", Pitch:", a->pitch, 2);
    FIELD_LABELED(", Yaw:", a->yaw, 2);
    FIELD_LABELED(", RollSp:", a->rollSp, 2);
    FIELD_LABELED(", PitchSp:", a->pitchSp, 2);
    FIELD_LABELED(", YawSp:", a->rateSp[2], 1);     /* yaw is flown as a rate: deg/s */

    p = PutStr(p, " | [MOT] ");
    FIELD_LABELED("M1:", o->motor[0], 0);
    FIELD_LABELED(", M2:", o->motor[1], 0);
    FIELD_LABELED(", M3:", o->motor[2], 0);
    FIELD_LABELED(", M4:", o->motor[3], 0);
    FIELD_LABELED(", Thr:", o->thrust, 4);
    FIELD_LABELED(", Hover:", o->hoverThrust, 4);
    FIELD_LABELED(", Sat:", o->sat & 0x1F, 0);
    if(o->sat & TLM_SAT_TUMBLE) p = PutStr(p, " TUMBLE-CUT");

    p = PutStr(p, " | [ALT] "); // <-- Đã sửa lỗi gõ nhầm ở đây
    FIELD_LABELED("Alt:", o->altitude, 2);
    FIELD_LABELED(", Baro:", o->baroAltitude, 2);
    FIELD_LABELED(", Vz:", o->vz, 2);
    FIELD_LABELED(", ClimbSp:", o->climbSp, 2);
    FIELD_LABELED(", AccelUp:", o->accelUp, 2);
    FIELD_LABELED(", Valid:", (o->sat & TLM_SAT_ALT_VALID) ? 1 : 0, 0);
    FIELD_LABELED(", BaroOk:", (s->valid & TLM_VALID_BARO) ? 1 : 0, 0);
    FIELD_LABELED(", BaroAge:", s->baroAgeMs, 0);
    FIELD_LABELED(", ABias:", s->accelBias, 2);

    p = PutStr(p, " | [TOF] ");
    FIELD_LABELED("Dist:", s->rangeMm, 3);
    FIELD_LABELED(", Height:", s->rangeHeight, 2);
    FIELD_LABELED(", Status:", s->rangeStatus, 0);
    FIELD_LABELED(", Age:", s->rangeAgeMs, 0);
    FIELD_LABELED(", Ok:", (s->valid & TLM_VALID_RANGE) ? 1 : 0, 0);
    FIELD_LABELED(", Tilt:", s->tiltDeg, 0);

    p = PutStr(p, " | [FLOW] ");
    FIELD_LABELED("X:", s->flowX, 0);
    FIELD_LABELED(", Y:", s->flowY, 0);
    FIELD_LABELED(", Dt:", s->flowDtMs, 0);
    FIELD_LABELED(", Squal:", s->squal, 0);
    FIELD_LABELED(", Age:", s->flowAgeMs, 0);
    FIELD_LABELED(", Ok:", (s->valid & TLM_VALID_FLOW) ? 1 : 0, 0);

    p = PutStr(p, " | [MISC] ");
    FIELD_LABELED("Vib:", o->vibration, 1);
    FIELD_LABELED(", DtMax:", o->rateDtMaxUs, 0);
    FIELD_LABELED(", RCLost:", o->rcLost, 0);
    p = PutStick(p);

    p = PutStr(p, "\r\n");
    return (uint16_t)(p - buf);
}

void TelemetryPrintTask(void *argument){
    static char line[768];
    tlmTask = xTaskGetCurrentTaskHandle();
    TLM_Write(tlmLegend, sizeof(tlmLegend) - 1);

    uint32_t   printed   = 0;
    TickType_t lastStale = 0;
    TickType_t last      = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TLM_PRINT_PERIOD_MS));

        TLM_Attitude_t a;
        TLM_Output_t   o;
        TLM_Sensor_t   s;
        uint32_t       count;
        
        taskENTER_CRITICAL();
        a     = rawAtt;
        o     = rawOut;
        s     = rawSen;
        count = outCount;
        taskEXIT_CRITICAL();

        TickType_t now = xTaskGetTickCount();

        if(count != printed){
            printed = count;
            TLM_WaitIdle();     /* the last line may still be on the wire */
            TLM_Write(line, Telemetry_FormatLine(line, &a, &o, &s, linkPct));
        }
        else if((now - telemetry.lastFrameTick) > pdMS_TO_TICKS(TLM_STALE_MS) &&
                (now - lastStale) > pdMS_TO_TICKS(1000)){
            lastStale = now;
            TLM_WaitIdle();
            char *p = PutStr(line, "# no telemetry, link ");
            p = PutU32(p, linkPct);
            p = PutStr(p, "%");
            p = PutStick(p);
            p = PutStr(p, "\r\n");
            TLM_Write(line, (uint16_t)(p - line));
        }
    }
}
