#include "telemetry.h"
#include "rc_protocol.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define TLM_PRINT_PERIOD_MS     1000U         /* poll; one line per new output frame (25 Hz) */
#define TLM_STALE_MS            500U
#define TLM_HEADER_EVERY        250U        /* lines (~10 s), for a terminal opened late */
#define TLM_UART_TIMEOUT_MS     100U

extern UART_HandleTypeDef huart1;

Telemetry_t telemetry;

/* raw frames for the printer, guarded by a critical section */
static TLM_Attitude_t rawAtt;
static TLM_Output_t   rawOut;
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
    telemetry.magBaroCalibrated = (s & TLM_STATE_MAGBARO_CAL) ? 1 : 0;
    telemetry.altitudeValid     = (s & TLM_STATE_ALT_VALID)   ? 1 : 0;
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
        telemetry.saturation    = f.sat;
        telemetry.altitude      = f.altitude     * 0.01f;
        telemetry.baroAltitude  = f.baroAltitude * 0.01f;
        telemetry.verticalSpeed = f.vz           * 0.01f;
        telemetry.climbSp       = f.climbSp      * 0.01f;
        telemetry.accelUp       = f.accelUp      * 0.01f;
        telemetry.vibration     = f.vibration    * 0.1f;
        telemetry.rateDtMaxUs   = f.rateDtMaxUs;
        telemetry.rcLost        = f.rcLost;
        telemetry.droneTimeMs   = f.time_ms;
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

static void TLM_Write(const char *s, uint16_t len){
    HAL_UART_Transmit(&huart1, (uint8_t *)s, len, TLM_UART_TIMEOUT_MS);
}

#define FIELD_FIXED(v, d)   do{ p = PutFixed(p, (int32_t)(v), (d)); *p++ = ','; }while(0)
#define FIELD_LABELED(label, v, d) \
    do{ p = PutStr(p, label); p = PutFixed(p, (int32_t)(v), (d)); }while(0)

static const char tlmHeader[] =
    "t_ms,link,arm,mode,flags,"
    "roll,pitch,yaw,rollSp,pitchSp,"
    "gx,gy,gz,gxSp,gySp,gzSp,"
    "tqR,tqP,tqY,"
    "m1,m2,m3,m4,thr,sat,"
    "alt,baroAlt,vz,climbSp,accUp,"
    "vib,dtMax,rcLost\r\n";

static const char tlmLegend[] =
    "# drone telemetry, 25 Hz. angles deg, rates deg/s, motors us, alt m, vz m/s, vib m/s^2, dtMax us\r\n"
    "# arm: 0 DISARMED 1 ARMING 2 ARMED_GROUND 3 TAKEOFF 4 AIRBORNE 5 LANDING 6 FAILSAFE\r\n"
    "# mode: 0 ANGLE 1 ALT_HOLD 2 POS_HOLD | flags: 1 imu cal, 2 mag+baro cal, 4 alt valid\r\n"
    "# sat: 1 roll/pitch cut, 2 yaw cut, 4 throttle at max, 8 throttle at min\r\n"
    "# motors: M1 front-right, M2 rear-right, M3 rear-left, M4 front-left\r\n";

static uint16_t Telemetry_FormatLine(char *buf, const TLM_Attitude_t *a, const TLM_Output_t *o, uint8_t link){
    char *p = buf;

    p = PutStr(p, "[SYS] ");
    FIELD_LABELED("Time:", xTaskGetTickCount() * portTICK_PERIOD_MS, 0);
    FIELD_LABELED("ms, Link:", link, 0);
    FIELD_LABELED("%, Arm:", o->state & TLM_STATE_ARM_MASK, 0);
    FIELD_LABELED(", Mode:", (o->state & TLM_STATE_MODE_MASK) >> TLM_STATE_MODE_SHIFT, 0);
    FIELD_LABELED(", Flags:", o->state >> 5, 0);

    p = PutStr(p, " | [ATT] ");
    FIELD_LABELED("Roll:", a->roll, 2);
    FIELD_LABELED(", Pitch:", a->pitch, 2);
    FIELD_LABELED(", Yaw:", a->yaw, 2);
    FIELD_LABELED(", RollSp:", a->rollSp, 2);
    FIELD_LABELED(", PitchSp:", a->pitchSp, 2);

    p = PutStr(p, " | [MOT] ");
    FIELD_LABELED("M1:", o->motor[0], 0);
    FIELD_LABELED(", M2:", o->motor[1], 0);
    FIELD_LABELED(", M3:", o->motor[2], 0);
    FIELD_LABELED(", M4:", o->motor[3], 0);
    FIELD_LABELED(", Thr:", o->thrust, 4);

    p = PutStr(p, " | [ALT] "); // <-- Đã sửa lỗi gõ nhầm ở đây
    FIELD_LABELED("Alt:", o->altitude, 2);
    FIELD_LABELED(", Baro:", o->baroAltitude, 2);
    FIELD_LABELED(", Vz:", o->vz, 2);

    p = PutStr(p, " | [MISC] ");
    FIELD_LABELED("Vib:", o->vibration, 1);
    FIELD_LABELED(", DtMax:", o->rateDtMaxUs, 0);
    FIELD_LABELED(", RCLost:", o->rcLost, 0);

    p = PutStr(p, "\r\n");
    return (uint16_t)(p - buf);
}

void TelemetryPrintTask(void *argument){
    static char line[512]; 

    TLM_Write(tlmLegend, sizeof(tlmLegend) - 1);

    uint32_t   printed   = 0;
    TickType_t lastStale = 0;
    TickType_t last      = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(TLM_PRINT_PERIOD_MS));

        TLM_Attitude_t a;
        TLM_Output_t   o;
        uint32_t       count;
        
        taskENTER_CRITICAL();
        a     = rawAtt;
        o     = rawOut;
        count = outCount;
        taskEXIT_CRITICAL();

        TickType_t now = xTaskGetTickCount();

        if(count != printed){
            printed = count;
            TLM_Write(line, Telemetry_FormatLine(line, &a, &o, linkPct));
        }
        else if((now - telemetry.lastFrameTick) > pdMS_TO_TICKS(TLM_STALE_MS) &&
                (now - lastStale) > pdMS_TO_TICKS(1000)){
            lastStale = now;
            char *p = PutStr(line, "# no telemetry, link ");
            p = PutU32(p, linkPct);
            p = PutStr(p, "%\r\n");
            TLM_Write(line, (uint16_t)(p - line));
        }
    }
}
