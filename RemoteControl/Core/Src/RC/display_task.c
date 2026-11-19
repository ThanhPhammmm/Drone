#include "display_task.h"
#include "ili9341.h"
#include "font8x16.h"
#include "telemetry.h"
#include "rc_tx_task.h"
#include "rc_protocol.h"
#include "joystick.h"
#include "nrf24_tx.h"
#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <math.h>

/* ---- screen: 40 columns x 15 rows of 8x16 characters ---- */
#define COLS                (ILI9341_WIDTH  / FONT_W)   /* 40 */
#define ROWS                (ILI9341_HEIGHT / FONT_H)   /* 15 */
#define ROW_HEADER          0
#define ROW_FOOTER          (ROWS - 1)

#define BTN_TICK_MS         5U      /* button sampled by its own task, never blocked by drawing */
#define BTN_DEBOUNCE_TICKS  4U      /* 20 ms stable before a change counts */
#define BTN_LONG_PRESS_MS   1000U
#define BTN_QUEUE_LEN       8U
#define DISP_REFRESH_MS     200U    /* 5 Hz */
#define DISP_REFRESH_RC_MS  100U    /* 10 Hz on the RC page */
#define DISP_STALE_MS       500U    /* telemetry older than this is shown as "--" */

#define LINK_LOW_PCT        70U
#define THRUST_HIGH         0.80f
#define VIB_WARN            8.0f
#define VIB_HIGH            15.0f
#define MOTOR_WARN_US       1900U
#define MOTOR_MAX_US        1990U

typedef enum{ PAGE_STATUS = 0, PAGE_RC, PAGE_ATTITUDE, PAGE_HEIGHT, PAGE_GPS, PAGE_COUNT } Page_t;
typedef enum{ BTN_SHORT = 1, BTN_LONG } BtnEvent_t;

static QueueHandle_t btnQueue;

static const char *const pageName[PAGE_COUNT] = { "STATUS", "RC", "ATTITUDE", "HEIGHT/POS", "GPS/LINK" };
static const char *const armName[7]  = { "DISARMED", "ARMING", "READY", "TAKEOFF", "FLYING", "LANDING", "FAILSAFE" };
static const char *const armShort[7] = { "OFF", "ARM", "RDY", "TKO", "FLY", "LND", "FS!" };
static const char *const modeName[3] = { "ANGLE", "ALT HOLD", "POS HOLD" };

typedef struct{
    char     ch[ROWS][COLS];
    uint16_t fg[ROWS][COLS];
    uint16_t bg[ROWS][COLS];
} Screen_t;

static Screen_t shown;      /* what the panel shows now */
static Screen_t next;       /* the page being built */

/* data for one refresh */
static Telemetry_t   tlm;
static RCTx_Status_t rc;
static uint8_t       rcValid;
static uint8_t       fresh;         /* telemetry is recent */

/* per-second statistics */
static uint8_t  packetsPerSec;
static uint16_t lostPerSec;
static uint8_t  linkMin60;
static uint8_t  linkMinSec[60];
static uint8_t  linkMinIdx;
static uint8_t  linkMinNow = 100;

/* ------------------------------------------------------------ text helpers */

static void Put(uint8_t col, uint8_t row, const char *s, uint16_t fg){
    while(*s && col < COLS){
        next.ch[row][col] = *s++;
        next.fg[row][col] = fg;
        col++;
    }
}

/* right-aligned fixed-point number: width characters, dec decimals, optional '+' */
static void FmtFixed(char *out, uint8_t width, float v, uint8_t dec, uint8_t plus){
    char tmp[16];
    uint8_t n = 0;
    if(isnan(v) || isinf(v)){
        tmp[n++] = '-'; tmp[n++] = '-';
    }
    else{
        static const uint32_t kPow10[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 };
        uint32_t scale = kPow10[dec];
        float a = fabsf(v);
        if(a > 999999.0f) a = 999999.0f;
        uint32_t q  = (uint32_t)(a * (float)scale + 0.5f);
        uint32_t ip = q / scale, fp = q % scale;
        char digits[12];
        uint8_t d = 0;
        do{ digits[d++] = (char)('0' + ip % 10U); ip /= 10U; } while(ip > 0U);
        if(v < 0.0f && q != 0U) tmp[n++] = '-';
        else if(plus)           tmp[n++] = '+';
        while(d > 0U) tmp[n++] = digits[--d];
        if(dec > 0U){
            tmp[n++] = '.';
            for(int8_t k = (int8_t)dec - 1; k >= 0; k--){
                tmp[n + (uint8_t)k] = (char)('0' + fp % 10U);
                fp /= 10U;
            }
            n += dec;
        }
    }
    uint8_t pad = (n < width) ? (uint8_t)(width - n) : 0U;
    memset(out, ' ', pad);
    memcpy(out + pad, tmp, n);
    out[pad + n] = '\0';
}

/* number from the drone: "--" in grey when the telemetry is stale */
static void PutF(uint8_t col, uint8_t row, uint8_t width, float v, uint8_t dec, uint8_t plus, uint16_t fg){
    char buf[20];
    if(!fresh){
        FmtFixed(buf, width, NAN, 0, 0);
        fg = C_GREY;
    }
    else{
        FmtFixed(buf, width, v, dec, plus);
    }
    Put(col, row, buf, fg);
}

/* number measured here on the remote: always valid */
static void PutL(uint8_t col, uint8_t row, uint8_t width, float v, uint8_t dec, uint8_t plus, uint16_t fg){
    char buf[20];
    FmtFixed(buf, width, v, dec, plus);
    Put(col, row, buf, fg);
}

/* latitude / longitude with 7 decimals (a float has only ~7 significant digits) */
static void PutCoord(uint8_t col, uint8_t row, double deg){
    if(!fresh){ Put(col, row, "--", C_GREY); return; }
    char buf[20];
    uint8_t n = 0;
    double a = fabs(deg);
    uint64_t q = (uint64_t)(a * 1e7 + 0.5);
    uint32_t ip = (uint32_t)(q / 10000000ULL), fp = (uint32_t)(q % 10000000ULL);
    if(deg < 0.0) buf[n++] = '-';
    char digits[4]; uint8_t d = 0;
    do{ digits[d++] = (char)('0' + ip % 10U); ip /= 10U; } while(ip > 0U && d < sizeof(digits));
    while(d > 0U) buf[n++] = digits[--d];
    buf[n++] = '.';
    for(int8_t k = 6; k >= 0; k--){ buf[n + (uint8_t)k] = (char)('0' + fp % 10U); fp /= 10U; }
    n += 7;
    buf[n] = '\0';
    Put(col, row, buf, C_WHITE);
}

static uint16_t ColorBelow(float v, float warn, float bad){
    return (v >= bad) ? C_RED : (v >= warn) ? C_YELLOW : C_GREEN;
}

/* [=====|     ] stick bar, value -1000..+1000, 11 cells, '|' = centre */
static void PutStickBar(uint8_t col, uint8_t row, int16_t v){
    char bar[14];
    bar[0] = '['; bar[12] = ']'; bar[13] = '\0';
    for(uint8_t i = 1; i <= 11; i++) bar[i] = ' ';
    int8_t pos = (int8_t)((v + RC_CHANNEL_MAX) * 10 / (2 * RC_CHANNEL_MAX));   /* 0..10 */
    for(int8_t i = (pos < 5 ? pos : 5); i <= (pos > 5 ? pos : 5); i++) bar[1 + i] = '=';
    bar[6] = '|';
    Put(col, row, bar, C_CYAN);
}

/* [=======    ] throttle bar, value 0..1000 */
static void PutThrottleBar(uint8_t col, uint8_t row, uint16_t v){
    char bar[14];
    bar[0] = '['; bar[12] = ']'; bar[13] = '\0';
    uint8_t fill = (uint8_t)((uint32_t)v * 11U / RC_THROTTLE_MAX);
    for(uint8_t i = 0; i < 11; i++) bar[1 + i] = (i < fill) ? '=' : ' ';
    Put(col, row, bar, C_CYAN);
}

/* ------------------------------------------------------------ pages */

static uint8_t Flying(void){
    return fresh && tlm.armState >= 3 && tlm.armState <= 6;
}

static uint8_t MaxMotor(void){
    uint16_t m = 0;
    for(uint8_t i = 0; i < 4; i++) if(tlm.motor[i] > m) m = tlm.motor[i];
    return Flying() && m >= MOTOR_MAX_US;
}

static void Header(Page_t page){
    char buf[8];
    Put(0, ROW_HEADER, "P", C_WHITE);
    buf[0] = (char)('1' + page); buf[1] = '/'; buf[2] = (char)('0' + PAGE_COUNT); buf[3] = '\0';
    Put(1, ROW_HEADER, buf, C_WHITE);
    Put(5, ROW_HEADER, pageName[page], C_WHITE);

    uint8_t link = tlm.linkQuality;
    Put(26, ROW_HEADER, "LINK", C_WHITE);
    PutL(30, ROW_HEADER, 4, link, 0, 0, link >= LINK_LOW_PCT ? C_GREEN : C_RED);
    Put(34, ROW_HEADER, "%", C_WHITE);
    if(fresh && tlm.armState < 7) Put(36, ROW_HEADER, armShort[tlm.armState], tlm.armState == 6 ? C_RED : C_WHITE);
    else                          Put(36, ROW_HEADER, "---", C_GREY);
}

static void Footer(void){
    const char *msg = NULL;
    uint8_t severe = 0;
    if(!fresh)                                      { msg = "NO TELEMETRY";            severe = 1; }
    else if(tlm.armState == 6)                      { msg = "FAILSAFE - LINK LOST";    severe = 1; }
    else if(MaxMotor())                             { msg = "MOTOR AT MAXIMUM";        severe = 1; }
    else if(tlm.linkQuality < LINK_LOW_PCT)         { msg = "LINK LOW"; }
    else if(Flying() && tlm.thrust > THRUST_HIGH)   { msg = "THRUST HIGH - LAND SOON"; }
    else if(tlm.vibration > VIB_HIGH)               { msg = "VIBRATION HIGH"; }
    else if(Flying() && rcValid && rc.lastSent.mode != tlm.mode){ msg = "MODE FALLBACK"; }
    else if(rcValid && (rc.lastSent.flags & RC_FLAG_ARM) && tlm.armState == 0 &&
            rc.lastSent.throttle > 20U)             { msg = "THROTTLE NOT LOW - CANNOT ARM"; }
    if(msg == NULL) return;

    uint16_t bg = severe ? C_RED : C_BLACK;
    uint16_t fg = severe ? C_WHITE : C_YELLOW;
    for(uint8_t c = 0; c < COLS; c++) next.bg[ROW_FOOTER][c] = bg;
    Put(0, ROW_FOOTER, msg, fg);
}

static void PageStatus(void){
    uint8_t r = 2;
    Put(0, r, "State   :", C_GREY);
    if(fresh && tlm.armState < 7){
        uint16_t c = (tlm.armState == 4) ? C_GREEN : (tlm.armState == 6) ? C_RED : C_WHITE;
        Put(10, r, armName[tlm.armState], c);
    }
    else Put(10, r, "--", C_GREY);

    r++;
    Put(0, r, "Mode req:", C_GREY);
    if(rcValid && rc.lastSent.mode < 3) Put(10, r, modeName[rc.lastSent.mode], C_WHITE);
    r++;
    Put(0, r, "Mode run:", C_GREY);
    if(fresh && tlm.mode < 3){
        uint16_t c = (rcValid && rc.lastSent.mode != tlm.mode) ? C_YELLOW : C_WHITE;
        Put(10, r, modeName[tlm.mode], c);
    }
    else Put(10, r, "--", C_GREY);

    r++;
    Put(0, r, "Height  :", C_GREY);  PutF(10, r, 6, tlm.aboveFloor, 2, 0, C_WHITE);  Put(17, r, "m", C_GREY);
    r++;
    Put(0, r, "Climb   :", C_GREY);  PutF(10, r, 6, tlm.verticalSpeed, 2, 1, C_WHITE);  Put(17, r, "m/s", C_GREY);
    r++;
    Put(0, r, "Pos X/Y :", C_GREY);  PutF(10, r, 6, tlm.x, 2, 1, C_WHITE);  Put(17, r, "/", C_GREY);
    PutF(19, r, 6, tlm.y, 2, 1, C_WHITE);  Put(26, r, "m", C_GREY);
    r++;
    Put(0, r, "Thrust  :", C_GREY);
    PutF(10, r, 6, tlm.thrust, 2, 0, tlm.thrust > THRUST_HIGH ? C_YELLOW : C_WHITE);
    Put(19, r, "Hover", C_GREY);  PutF(25, r, 5, tlm.hoverThrust, 2, 0, C_WHITE);
    r++;
    Put(0, r, "Vib     :", C_GREY);  PutF(10, r, 6, tlm.vibration, 1, 0, ColorBelow(tlm.vibration, VIB_WARN, VIB_HIGH));
    Put(17, r, "m/s2", C_GREY);
    r++;
    Put(0, r, "Sensors :", C_GREY);
    {
        uint8_t ok[5] = { tlm.imuCalibrated, tlm.baroCalibrated,
                          (tlm.sensorValid & TLM_VALID_RANGE) ? 1U : 0U,
                          (tlm.sensorValid & TLM_VALID_FLOW)  ? 1U : 0U,
                          tlm.gpsFixOk };
        static const char *const name[5] = { "IMU", "BARO", "TOF", "FLOW", "GPS" };
        uint8_t c = 10;
        for(uint8_t i = 0; i < 5; i++){
            Put(c, r, name[i], !fresh ? C_GREY : ok[i] ? C_GREEN : C_RED);
            c += (uint8_t)strlen(name[i]) + 1U;
        }
    }
    r++;
    Put(0, r, "RC lost :", C_GREY);  PutF(10, r, 6, tlm.rcLost, 0, 0, C_WHITE);
    PutL(17, r, 5, lostPerSec, 0, 1, lostPerSec > 5U ? C_RED : C_WHITE);  Put(22, r, "/s", C_GREY);
    r++;
    Put(0, r, "Calib   :", C_GREY);
    Put(10, r, "IMU",  !fresh ? C_GREY : tlm.imuCalibrated  ? C_GREEN : C_RED);
    Put(14, r, "BARO", !fresh ? C_GREY : tlm.baroCalibrated ? C_GREEN : C_RED);
    Put(19, r, "MAG",  !fresh ? C_GREY : tlm.magCalibrated  ? C_GREEN : C_GREY);
}

static void PageRc(void){
    /* everything here is measured on the remote: valid without a link */
    Put(0, 2, "       SENT   RAW", C_GREY);
    static const char *const label[4] = { "Thr  :", "Roll :", "Pitch:", "Yaw  :" };
    uint16_t raw[4] = { JOY_ThrottleRaw(), JOY_RollRaw(), JOY_PitchRaw(), JOY_YawRaw() };
    for(uint8_t i = 0; i < 4; i++){
        uint8_t r = 3 + i;
        Put(0, r, label[i], C_GREY);
        if(!rcValid) continue;
        int32_t v = (i == 0) ? (int32_t)rc.lastSent.throttle :
                    (i == 1) ? rc.lastSent.roll : (i == 2) ? rc.lastSent.pitch : rc.lastSent.yaw;
        PutL(6, r, 6, (float)v, 0, i != 0, C_WHITE);
        PutL(13, r, 5, raw[i], 0, 0, C_GREY);
        if(i == 0) PutThrottleBar(20, r, (uint16_t)v);
        else       PutStickBar(20, r, (int16_t)v);
    }
    if(!rcValid) return;

    Put(0, 8, "Arm sw :", C_GREY);
    uint8_t armSw = (rc.lastSent.flags & RC_FLAG_ARM) ? 1U : 0U;
    Put(9, 8, armSw ? "ON " : "OFF", armSw ? C_GREEN : C_WHITE);
    Put(0, 9, "Mode sw:", C_GREY);
    if(rc.lastSent.mode < 3) Put(9, 9, modeName[rc.lastSent.mode], C_WHITE);
    Put(0, 10, "Packets:", C_GREY);  PutL(9, 10, 3, packetsPerSec, 0, 0, C_WHITE);  Put(12, 10, "/s", C_GREY);
    Put(16, 10, "acked", C_GREY);    PutL(22, 10, 3, tlm.linkQuality, 0, 0, tlm.linkQuality >= LINK_LOW_PCT ? C_GREEN : C_RED);
    Put(25, 10, "%", C_GREY);
    Put(0, 11, "Radio  :", C_GREY);
    Put(9, 11, rc.radioOk ? "OK  " : "FAIL", rc.radioOk ? C_GREEN : C_RED);
    Put(16, 11, "CH", C_GREY);       PutL(19, 11, 3, NRF24_CHANNEL, 0, 0, C_WHITE);
    Put(0, 12, "Lost   :", C_GREY);  PutF(9, 12, 6, tlm.rcLost, 0, 0, C_WHITE);
    PutL(16, 12, 5, lostPerSec, 0, 1, lostPerSec > 5U ? C_RED : C_WHITE);  Put(21, 12, "/s", C_GREY);
}

static void PageAttitude(void){
    Put(0, 2, "         ANGLE    SET", C_GREY);
    Put(0, 3, "Roll  :", C_GREY);  PutF(8, 3, 7, tlm.roll,  1, 1, C_WHITE);  PutF(16, 3, 7, tlm.rollSp,  1, 1, C_CYAN);
    Put(0, 4, "Pitch :", C_GREY);  PutF(8, 4, 7, tlm.pitch, 1, 1, C_WHITE);  PutF(16, 4, 7, tlm.pitchSp, 1, 1, C_CYAN);
    Put(0, 5, "Yaw   :", C_GREY);  PutF(8, 5, 7, tlm.yaw,   1, 1, C_WHITE);
    Put(16, 5, "rate", C_GREY);    PutF(21, 5, 5, tlm.yawRateSp, 0, 1, C_CYAN);
    Put(0, 6, "Tilt  :", C_GREY);  PutF(8, 6, 7, tlm.tiltDeg, 0, 0, C_WHITE);  Put(16, 6, "deg", C_GREY);

    /* motors laid out as on the frame, seen from above: M4 front-left, M1 front-right */
    Put(0, 8, "Motors (us)", C_GREY);
    static const uint8_t mIdx[4] = { 3, 0, 2, 1 };          /* M4 M1 / M3 M2 */
    static const char *const mName[4] = { "M4", "M1", "M3", "M2" };
    for(uint8_t k = 0; k < 4; k++){
        uint8_t r = (k < 2) ? 9 : 10, c = (k % 2) ? 14 : 1;
        uint16_t us = tlm.motor[mIdx[k]];
        Put(c, r, mName[k], C_GREY);
        PutF(c + 3, r, 5, us, 0, 0, ColorBelow(us, MOTOR_WARN_US, MOTOR_MAX_US));
    }
    Put(27, 9, "front", C_GREY);
    Put(27, 10, "back", C_GREY);

    Put(0, 11, "Thrust:", C_GREY);  PutF(8, 11, 5, tlm.thrust, 2, 0, tlm.thrust > THRUST_HIGH ? C_YELLOW : C_WHITE);
    Put(15, 11, "Hover", C_GREY);   PutF(21, 11, 5, tlm.hoverThrust, 2, 0, C_WHITE);

    /* imbalance from the part of each pulse above 1000 us */
    float u[4];
    for(uint8_t i = 0; i < 4; i++) u[i] = (float)tlm.motor[i] - 1000.0f;
    float mean = (u[0] + u[1] + u[2] + u[3]) * 0.25f;
    float fb = NAN, rl = NAN;
    if(mean > 50.0f){
        fb = ((u[0] + u[3]) - (u[1] + u[2])) * 0.5f / mean * 100.0f;    /* front - back */
        rl = ((u[0] + u[1]) - (u[2] + u[3])) * 0.5f / mean * 100.0f;    /* right - left */
    }
    Put(0, 12, "Imbal F-B", C_GREY);  PutF(10, 12, 6, fb, 1, 1, C_WHITE);  Put(16, 12, "%", C_GREY);
    Put(19, 12, "R-L", C_GREY);       PutF(23, 12, 6, rl, 1, 1, C_WHITE);  Put(29, 12, "%", C_GREY);
    Put(0, 13, "Vib   :", C_GREY);    PutF(8, 13, 5, tlm.vibration, 1, 0, ColorBelow(tlm.vibration, VIB_WARN, VIB_HIGH));
    Put(14, 13, "m/s2", C_GREY);
}

static void PageHeight(void){
    Put(0, 2, "Height :", C_GREY);  PutF(9, 2, 6, tlm.aboveFloor, 2, 0, C_WHITE);  Put(16, 2, "m above floor", C_GREY);
    Put(0, 3, "Alt    :", C_GREY);  PutF(9, 3, 6, tlm.altitude, 2, 1, C_WHITE);
    Put(16, 3, "set", C_GREY);      PutF(20, 3, 6, tlm.altitudeSp, 2, 1, C_CYAN);  Put(27, 3, "m", C_GREY);
    Put(0, 4, "Climb  :", C_GREY);  PutF(9, 4, 6, tlm.verticalSpeed, 2, 1, C_WHITE);
    Put(16, 4, "set", C_GREY);      PutF(20, 4, 6, tlm.climbSp, 2, 1, C_CYAN);  Put(27, 4, "m/s", C_GREY);
    Put(0, 5, "ToF    :", C_GREY);
    uint8_t tofOk = (tlm.sensorValid & TLM_VALID_RANGE) ? 1U : 0U;
    PutF(9, 5, 6, tlm.rangeHeight, 2, 0, tofOk ? C_WHITE : C_RED);  Put(16, 5, "m", C_GREY);
    Put(19, 5, fresh ? (tofOk ? "ok  " : "LOST") : "--  ", !fresh ? C_GREY : tofOk ? C_GREEN : C_RED);

    Put(0, 7, "Pos X  :", C_GREY);  PutF(9, 7, 6, tlm.x, 2, 1, C_WHITE);
    Put(16, 7, "set", C_GREY);      PutF(20, 7, 6, tlm.xSp, 2, 1, C_CYAN);  Put(27, 7, "m", C_GREY);
    Put(0, 8, "Pos Y  :", C_GREY);  PutF(9, 8, 6, tlm.y, 2, 1, C_WHITE);
    Put(16, 8, "set", C_GREY);      PutF(20, 8, 6, tlm.ySp, 2, 1, C_CYAN);  Put(27, 8, "m", C_GREY);
    Put(0, 9, "Vel X/Y:", C_GREY);  PutF(9, 9, 6, tlm.vx, 2, 1, C_WHITE);  Put(16, 9, "/", C_GREY);
    PutF(18, 9, 6, tlm.vy, 2, 1, C_WHITE);  Put(25, 9, "m/s", C_GREY);
    Put(0, 10, "Source :", C_GREY);
    static const char *const src[3] = { "NONE", "FLOW", "GPS " };
    if(fresh && tlm.posSource < 3) Put(9, 10, src[tlm.posSource], tlm.posSource ? C_GREEN : C_RED);
    else                           Put(9, 10, "--", C_GREY);
    Put(0, 11, "Flow   :", C_GREY);
    Put(9, 11, "X", C_GREY);  PutF(10, 11, 5, tlm.flowX, 0, 1, C_WHITE);
    Put(16, 11, "Y", C_GREY); PutF(17, 11, 5, tlm.flowY, 0, 1, C_WHITE);
    Put(23, 11, "Q", C_GREY); PutF(24, 11, 4, tlm.squal, 0, 0, tlm.squal >= 100 ? C_GREEN : tlm.squal >= 25 ? C_YELLOW : C_RED);
    float ex = tlm.x - tlm.xSp, ey = tlm.y - tlm.ySp;
    Put(0, 12, "Err XY :", C_GREY);  PutF(9, 12, 6, sqrtf(ex * ex + ey * ey), 2, 0, C_WHITE);  Put(16, 12, "m", C_GREY);
}

static void PageGps(void){
    static const char *const fixName[6] = { "NONE", "DR  ", "2D  ", "3D  ", "3D+DR", "TIME" };
    Put(0, 2, "Fix    :", C_GREY);
    if(fresh && tlm.gpsFound){
        uint8_t f = (tlm.gpsFix < 6) ? tlm.gpsFix : 0;
        Put(9, 2, fixName[f], (f == 3 || f == 4) ? C_GREEN : (f == 2) ? C_YELLOW : C_RED);
    }
    else Put(9, 2, fresh ? "NO GPS" : "--", fresh ? C_RED : C_GREY);
    Put(16, 2, "Sats", C_GREY);  PutF(21, 2, 3, tlm.gpsSats, 0, 0, tlm.gpsSats >= 6 ? C_GREEN : C_YELLOW);
    Put(0, 3, "H acc  :", C_GREY);  PutF(9, 3, 6, tlm.hAcc, 1, 0, tlm.hAcc <= 3.0f ? C_GREEN : C_YELLOW);  Put(16, 3, "m", C_GREY);
    Put(19, 3, "S acc", C_GREY);    PutF(25, 3, 5, tlm.sAcc, 1, 0, C_WHITE);
    Put(0, 4, "Lat    :", C_GREY);  PutCoord(9, 4, tlm.lat);
    Put(0, 5, "Lon    :", C_GREY);  PutCoord(9, 5, tlm.lon);
    Put(0, 6, "Vel N/E:", C_GREY);  PutF(9, 6, 6, tlm.velN, 2, 1, C_WHITE);  Put(16, 6, "/", C_GREY);
    PutF(18, 6, 6, tlm.velE, 2, 1, C_WHITE);  Put(25, 6, "m/s", C_GREY);
    Put(0, 7, "Aligned:", C_GREY);
    Put(9, 7, !fresh ? "--" : tlm.gpsAligned ? "YES" : "NO ", !fresh ? C_GREY : tlm.gpsAligned ? C_GREEN : C_YELLOW);
    Put(16, 7, "yaw off", C_GREY);  PutF(24, 7, 7, tlm.yawOffset, 1, 1, C_WHITE);

    Put(0, 9, "Link   :", C_GREY);
    PutL(9, 9, 3, tlm.linkQuality, 0, 0, tlm.linkQuality >= LINK_LOW_PCT ? C_GREEN : C_RED);  Put(12, 9, "%", C_GREY);
    Put(16, 9, "min 60s", C_GREY);
    PutL(24, 9, 3, linkMin60, 0, 0, linkMin60 >= LINK_LOW_PCT ? C_GREEN : C_RED);  Put(27, 9, "%", C_GREY);
    Put(0, 10, "Lost   :", C_GREY);  PutF(9, 10, 6, tlm.rcLost, 0, 0, C_WHITE);
    PutL(16, 10, 5, lostPerSec, 0, 1, lostPerSec > 5U ? C_RED : C_WHITE);  Put(21, 10, "/s", C_GREY);
    Put(0, 11, "Frames :", C_GREY);  PutL(9, 11, 8, (float)tlm.frames, 0, 0, C_WHITE);
}

/* ------------------------------------------------------------ refresh */

static void ClearScreen(Screen_t *s){
    for(uint8_t r = 0; r < ROWS; r++){
        uint16_t bg = (r == ROW_HEADER) ? C_NAVY : C_BLACK;
        for(uint8_t c = 0; c < COLS; c++){
            s->ch[r][c] = ' ';
            s->fg[r][c] = C_WHITE;
            s->bg[r][c] = bg;
        }
    }
}

/* send the cells that differ, in runs of the same colours */
static void Flush(void){
    for(uint8_t r = 0; r < ROWS; r++){
        uint8_t c = 0;
        while(c < COLS){
            if(next.ch[r][c] == shown.ch[r][c] && next.fg[r][c] == shown.fg[r][c] && next.bg[r][c] == shown.bg[r][c]){
                c++;
                continue;
            }
            uint8_t start = c;
            uint16_t fg = next.fg[r][c], bg = next.bg[r][c];
            while(c < COLS && next.fg[r][c] == fg && next.bg[r][c] == bg &&
                  (next.ch[r][c] != shown.ch[r][c] || next.fg[r][c] != shown.fg[r][c] || next.bg[r][c] != shown.bg[r][c])){
                c++;
            }
            ILI9341_DrawText(start * FONT_W, r * FONT_H, &next.ch[r][start], c - start, fg, bg);
            for(uint8_t k = start; k < c; k++){
                shown.ch[r][k] = next.ch[r][k];
                shown.fg[r][k] = next.fg[r][k];
                shown.bg[r][k] = next.bg[r][k];
            }
        }
    }
}

static void Snapshot(void){
    taskENTER_CRITICAL();                   /* the RC task writes telemetry while it runs */
    tlm = telemetry;
    taskEXIT_CRITICAL();
    rcValid = RCTx_GetStatus(&rc);
    fresh = (tlm.frames > 0U) &&
            ((xTaskGetTickCount() - tlm.lastFrameTick) < pdMS_TO_TICKS(DISP_STALE_MS));
}

static void Refresh(Page_t page){
    Snapshot();
    ClearScreen(&next);
    Header(page);
    switch(page){
    case PAGE_STATUS:   PageStatus();   break;
    case PAGE_RC:       PageRc();       break;
    case PAGE_ATTITUDE: PageAttitude(); break;
    case PAGE_HEIGHT:   PageHeight();   break;
    case PAGE_GPS:      PageGps();      break;
    default:            break;
    }
    Footer();
    Flush();
}

/* once a second: packets sent, packets lost, lowest link quality over the last minute */
static void UpdateStats(void){
    static uint8_t  lastSeq;
    static uint16_t lastLost;
    static uint8_t  first = 1;

    if(rcValid){
        packetsPerSec = (uint8_t)(rc.lastSent.seq - lastSeq);
        lastSeq = rc.lastSent.seq;
    }
    lostPerSec = first ? 0U : (uint16_t)(tlm.rcLost - lastLost);
    lastLost = tlm.rcLost;
    if(first){ packetsPerSec = 0; first = 0; }

    linkMinSec[linkMinIdx] = linkMinNow;
    linkMinIdx = (uint8_t)((linkMinIdx + 1U) % 60U);
    linkMinNow = 100;
    linkMin60 = 100;
    for(uint8_t i = 0; i < 60; i++) if(linkMinSec[i] < linkMin60) linkMin60 = linkMinSec[i];
}

static void ButtonTask(void *argument){
    (void)argument;
    uint8_t    stable = 0, debounce = 0, longSent = 0;
    uint32_t   heldMs = 0;
    TickType_t wake = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(BTN_TICK_MS));

        uint8_t pressed = (HAL_GPIO_ReadPin(DISP_BTN_PORT, DISP_BTN_PIN) == GPIO_PIN_RESET);
        if(pressed != stable){
            if(++debounce >= BTN_DEBOUNCE_TICKS){
                stable   = pressed;
                debounce = 0;
                if(stable){
                    heldMs   = 0;
                    longSent = 0;
                }
                else if(!longSent){
                    BtnEvent_t ev = BTN_SHORT;          /* released before 1 s */
                    xQueueSend(btnQueue, &ev, 0);
                }
            }
        }
        else{
            debounce = 0;
        }

        if(stable && !longSent){
            heldMs += BTN_TICK_MS;
            if(heldMs >= BTN_LONG_PRESS_MS){
                longSent = 1;
                BtnEvent_t ev = BTN_LONG;               /* fires while still held */
                xQueueSend(btnQueue, &ev, 0);
            }
        }
    }
}

void DisplayTask(void *argument){
    (void)argument;

    for(uint8_t i = 0; i < 60; i++) linkMinSec[i] = 100;

    btnQueue = xQueueCreate(BTN_QUEUE_LEN, sizeof(BtnEvent_t));
    configASSERT(btnQueue != NULL);
    BaseType_t ret = xTaskCreate(ButtonTask, "BTN", STACK_BTN, NULL, TASK_PRIO_BTN, NULL);
    configASSERT(ret == pdPASS);

    ILI9341_Init();
    ClearScreen(&shown);
    for(uint8_t c = 0; c < COLS; c++) shown.bg[ROW_HEADER][c] = C_BLACK;   /* panel is all black after init */

    Page_t     page        = PAGE_STATUS;
    TickType_t lastRefresh = xTaskGetTickCount() - pdMS_TO_TICKS(DISP_REFRESH_MS);
    TickType_t lastStats   = xTaskGetTickCount();

    while(1){
        /* sleep until the next refresh is due, or wake at once on a button event */
        uint32_t   period  = (page == PAGE_RC) ? DISP_REFRESH_RC_MS : DISP_REFRESH_MS;
        TickType_t elapsed = xTaskGetTickCount() - lastRefresh;
        TickType_t wait    = (elapsed >= pdMS_TO_TICKS(period)) ? 0 : pdMS_TO_TICKS(period) - elapsed;

        BtnEvent_t ev;
        if(xQueueReceive(btnQueue, &ev, wait) == pdTRUE){
            if(ev == BTN_SHORT) page = (Page_t)((page + 1) % PAGE_COUNT);
            else                page = PAGE_STATUS;
        }

        uint8_t lq = telemetry.linkQuality;             /* lowest link quality this second */
        if(lq < linkMinNow) linkMinNow = lq;

        if((xTaskGetTickCount() - lastStats) >= pdMS_TO_TICKS(1000)){
            lastStats += pdMS_TO_TICKS(1000);
            UpdateStats();
        }

        lastRefresh = xTaskGetTickCount();
        Refresh(page);
    }
}
