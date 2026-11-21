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

#define BTN_TICK_MS         5U      /* buttons sampled by their own task, never blocked by drawing */
#define BTN_DEBOUNCE_TICKS  10U      /* 20 ms stable before a change counts */
#define BTN_QUEUE_LEN       6U
#define DISP_REFRESH_MS     200U    /* 5 Hz */
#define DISP_REFRESH_RC_MS  100U    /* 10 Hz on the RC page */
#define DISP_STALE_MS       500U    /* telemetry older than this is shown as "--" */

#define LINK_LOW_PCT        70U
#define THRUST_HIGH         0.80f
#define VIB_WARN            8.0f
#define VIB_HIGH            15.0f
#define MOTOR_WARN_US       1900U
#define MOTOR_MAX_US        1990U
#define CAL_STALE_MS        1500U   /* no calibration frame this long: the drone is armed, or out of reach */
#define CAL_ASK_MS          2000U   /* "STARTING..." at most this long after a start */
#define CAL_ASK_FRAMES      3U      /* calibration frames until the drone's answer shows */

typedef enum{ PAGE_STATUS = 0, PAGE_RC, PAGE_ATTITUDE, PAGE_HEIGHT, PAGE_GPS, PAGE_CALIB, PAGE_COUNT } Page_t;
typedef enum{ BTN_PREV = 0, BTN_NEXT, BTN_OK, BTN_COUNT } BtnEvent_t;

static QueueHandle_t btnQueue;

static const char *const pageName[PAGE_COUNT] = { "STATUS", "RC", "ATTITUDE", "HEIGHT/POS", "GPS/LINK", "CALIBRATE" };
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

/* 1 = a warning took the footer */
static uint8_t Footer(Page_t page){
    const char *msg = NULL;
    uint8_t severe = 0;
    if(!fresh)                                      { msg = "NO TELEMETRY";            severe = 1; }
    else if(tlm.armState == 6)                      { msg = "FAILSAFE - LINK LOST";    severe = 1; }
    else if(MaxMotor())                             { msg = "MOTOR AT MAXIMUM";        severe = 1; }
    else if(tlm.linkQuality < LINK_LOW_PCT)         { msg = "LINK LOW"; }
    else if(Flying() && tlm.thrust > THRUST_HIGH)   { msg = "THRUST HIGH - LAND SOON"; }
    else if(tlm.vibration > VIB_HIGH)               { msg = "VIBRATION HIGH"; }
    else if(Flying() && rcValid && rc.lastSent.mode != tlm.mode){ msg = "MODE FALLBACK"; }
    else if(page != PAGE_CALIB && rcValid && (rc.lastSent.flags & RC_FLAG_CAL_MODE))
                                                    { msg = "CAL SWITCH ON - CANNOT ARM"; }
    else if(rcValid && (rc.lastSent.flags & RC_FLAG_ARM) && tlm.armState == 0 &&
            rc.lastSent.throttle > 20U)             { msg = "THROTTLE NOT LOW - CANNOT ARM"; }
    if(msg == NULL) return 0;

    uint16_t bg = severe ? C_RED : C_BLACK;
    uint16_t fg = severe ? C_WHITE : C_YELLOW;
    for(uint8_t c = 0; c < COLS; c++) next.bg[ROW_FOOTER][c] = bg;
    Put(0, ROW_FOOTER, msg, fg);
    return 1;
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
    Put(16, 8, "Cal sw :", C_GREY);
    uint8_t calSw = (rc.lastSent.flags & RC_FLAG_CAL_MODE) ? 1U : 0U;
    Put(25, 8, calSw ? "ON " : "OFF", calSw ? C_YELLOW : C_WHITE);
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

/* ------------------------------------------------------------ page 6: calibration
 * PREV / NEXT: move the arrow. OK: start it (EXIT: back to page 1), or cancel
 * the one running. The drone measures and uses the result at once, until
 * power-off; failed or cancelled keeps the values from before. */

#define CAL_MENU_EXIT       RC_CAL_COUNT            /* the line after the four sensors */
#define CAL_MENU_COUNT      (RC_CAL_COUNT + 1)
#define ROW_CAL_MENU        1                       /* rows 1-5 */
#define ROW_CAL_HELP        7                       /* rows 7-9 */
#define ROW_CAL_STATUS      10
#define ROW_CAL_BAR         11
#define ROW_CAL_RESULT      12                      /* rows 12-13 */

static const char *const calName[CAL_MENU_COUNT] = { "IMU", "BARO", "COMPASS", "FLOW", "EXIT" };
static const char *const calWhat[CAL_MENU_COUNT] = {
    "gyro + level", "ground height", "heading offsets", "camera scale", "back to page 1"
};
/* how to get an accurate result: up to 3 screens of 3 lines, shown in turn */
#define CAL_TIP_SCREENS     3
#define CAL_TIP_MS          4000U   /* each screen this long */
static const char *const calTips[CAL_MENU_COUNT][CAL_TIP_SCREENS][3] = {
    {   /* IMU */
        { "1. Props off. Power on and wait ~1 min", "   so the sensor is warm.",               "2. Stand it on its legs on a level" },
        { "   table: check with a spirit level or", "   a phone, the table becomes level.",   "3. No touching, no fan, no bumping." },
        { "4. Press OK and step away.",             "   Takes about 20 s.",                   "" },
    },
    {   /* BARO */
        { "1. Put the drone where it takes off,",   "   out of any draught (fan, window,",    "   air conditioner)." },
        { "2. Do it just before flying: the air",   "   pressure drifts with the weather.",   "3. Press OK. Takes about 2 s." },
        { NULL, NULL, NULL },
    },
    {   /* COMPASS */
        { "1. Props off, battery in as in flight",  "   (its wires bend the field too).",     "2. Go 1 m from metal, PCs, phones," },
        { "   speakers. Outdoors is best.",         "3. Press OK, then turn the drone",       "   slowly for 30 s:" },
        { "   nose down + full turn, nose up +",    "   full turn, each side down + turn,",   "   upside down, until X Y Z are 100%." },
    },
    {   /* FLOW */
        { "1. Props off. Floor with a pattern and", "   good light, no plain shiny tiles:",   "   page 4 Flow Q should be over 100." },
        { "2. Hold the drone level ~1 m above it,", "   camera down, away from your feet.",   "3. Press OK, then rock it:" },
        { "   left-right ~20 deg, ~1 per second,",  "   then forward-back the same way.",     "   Pivot on the camera, do not slide." },
    },
    {   /* EXIT */
        { "Press OK to go back to page 1.",        "",                                       "" },
        { NULL, NULL, NULL },
        { NULL, NULL, NULL },
    },
};

/* while it runs: what to do now */
static const char *const calDoing[RC_CAL_COUNT][3] = {
    { "Do not touch the drone or the table.",   "Keep the room still: no fan, no steps.",  "Moving it ends this with MOVED." },
    { "Keep it still, no air blowing on it.",   "",                                        "" },
    { "Turn it slowly through every direction:", "nose down/up, each side down, upside",   "down. Watch X Y Z below reach 100 %." },
    { "Rock left-right until roll is 100 %,",   "then forward-back until pitch is 100 %.", "Smooth, ~20 deg, camera over one spot." },
};

static uint8_t CalTipScreens(uint8_t item){
    uint8_t n = 0;
    while(n < CAL_TIP_SCREENS && calTips[item][n][0] != NULL) n++;
    return n;
}

static void Snapshot(void);

static uint8_t    calSel;               /* the arrow: RC_CalItem_t, or CAL_MENU_EXIT */
static uint8_t    calTipScreen;         /* the tips screen shown now */
static TickType_t calTipTick;
static uint8_t    calAsked;             /* a start was sent, the drone's answer is not in yet */
static uint8_t    calAskItem;
static TickType_t calAskTick;
static uint32_t   calAskFrames;

static uint8_t CalFresh(void){
    return tlm.calFrames > 0U && (xTaskGetTickCount() - tlm.calTick) < pdMS_TO_TICKS(CAL_STALE_MS);
}

static uint8_t CalRunning(void){
    return CalFresh() && tlm.calState == TLM_CAL_RUNNING;
}

static uint8_t CalAsking(void){
    if(!calAsked) return 0;
    if((xTaskGetTickCount() - calAskTick) >= pdMS_TO_TICKS(CAL_ASK_MS) ||
       (tlm.calFrames - calAskFrames) >= CAL_ASK_FRAMES || CalRunning()){
        calAsked = 0;
    }
    return calAsked;
}

/* why a start would be refused, or NULL */
static const char *CalBlocked(void){
    if(!rcValid || !(rc.lastSent.flags & RC_FLAG_CAL_MODE)) return "TURN CAL SWITCH ON";
    if(!fresh)                                              return "NO LINK TO THE DRONE";
    if(tlm.armState != 0)                                   return "DISARM FIRST";
    if(rcValid && (rc.lastSent.flags & RC_FLAG_ARM))        return "ARM SWITCH OFF FIRST";
    if(!CalFresh())                                         return "NO ANSWER FROM DRONE";
    return NULL;
}

static const char *CalReasonText(uint8_t r){
    switch(r){
    case TLM_CAL_ARMED:         return "DISARM FIRST";
    case TLM_CAL_MOVED:         return "MOVED, KEEP IT STILL";
    case TLM_CAL_TURN_MORE:     return "TURN IT MORE";
    case TLM_CAL_ROCK_MORE:     return "ROCK IT MORE";
    case TLM_CAL_BAD_FIT:       return "NOISY - SEE TIPS 1";
    case TLM_CAL_WRONG_SIGN:    return "AXIS SIGN WRONG";
    case TLM_CAL_AXES_SWAPPED:  return "X/Y AXES SWAPPED";
    case TLM_CAL_UNEVEN:        return "AXES DISAGREE, AGAIN";
    case TLM_CAL_OUT_OF_RANGE:  return "VALUE OUT OF RANGE";
    case TLM_CAL_CANCELLED:     return "CANCELLED";
    case TLM_CAL_NO_SENSOR:     return "NO SENSOR";
    case TLM_CAL_BUSY:          return "BUSY, TRY AGAIN";
    case TLM_CAL_SWITCH_OFF:    return "CAL SWITCH WAS OFF";
    default:                    return "?";
    }
}

static uint16_t ColorPct(int16_t pct){
    return (pct >= 100) ? C_GREEN : C_YELLOW;
}

/* [####################] 100% */
static void CalBar(uint8_t row, uint8_t pct, uint16_t fill){
    if(pct > 100U) pct = 100U;
    uint8_t n = (uint8_t)((pct * 20U + 50U) / 100U);
    Put(0, row, "[", C_GREY);
    for(uint8_t i = 0; i < 20U; i++) Put((uint8_t)(1U + i), row, (i < n) ? "#" : ".", (i < n) ? fill : C_GREY);
    Put(21, row, "]", C_GREY);
    PutL(23, row, 3, pct, 0, 0, C_WHITE);
    Put(26, row, "%", C_GREY);
}

/* the numbers for one item, rows 12-13; r is TLM_Calib_t.result */
static void CalResult(uint8_t item, uint8_t state, uint8_t reason, const int16_t *r){
    uint8_t row = ROW_CAL_RESULT;
    uint8_t done = (state == TLM_CAL_DONE);

    switch(item){
    case RC_CAL_IMU:
        if(!done) return;
        Put(0, row, "gyro", C_GREY);
        /* columns 5-13, 15-23, 25-33: a value is at most 9 characters, so one blank between */
        PutL(5,  row, 9, r[0] * 1e-6f, 6, 0, C_WHITE);
        PutL(15, row, 9, r[1] * 1e-6f, 6, 0, C_WHITE);
        PutL(25, row, 9, r[2] * 1e-6f, 6, 0, C_WHITE);
        Put(35, row, "rad/s", C_GREY);
        Put(0, row + 1, "accel", C_GREY);
        PutL(5,  row + 1, 9, r[3] * 1e-4f, 4, 1, C_WHITE);
        PutL(15, row + 1, 9, r[4] * 1e-4f, 4, 1, C_WHITE);
        PutL(25, row + 1, 9, r[5] * 1e-4f, 4, 1, C_WHITE);
        Put(35, row + 1, "m/s2", C_GREY);
        break;

    case RC_CAL_BARO:
        if(!done) return;
        Put(0, row, "ground", C_GREY);
        PutL(7, row, 7, r[0] * 0.1f, 1, 0, C_WHITE);
        Put(15, row, "hPa", C_GREY);
        Put(0, row + 1, "this height is 0 m now", C_GREY);
        break;

    case RC_CAL_COMPASS:
        if(done){
            Put(0, row, "offset", C_GREY);
            Put(7, row, "X", C_GREY);   PutL(9,  row, 6, r[0], 0, 1, C_WHITE);
            Put(16, row, "Y", C_GREY);  PutL(18, row, 6, r[1], 0, 1, C_WHITE);
            Put(25, row, "Z", C_GREY);  PutL(27, row, 6, r[2], 0, 1, C_WHITE);
            Put(34, row, "raw", C_GREY);
        }
        else if(state == TLM_CAL_RUNNING || reason == TLM_CAL_TURN_MORE){
            Put(0, row, "turned", C_GREY);
            Put(7, row, "X", C_GREY);   PutL(9,  row, 3, r[0], 0, 0, ColorPct(r[0]));  Put(12, row, "%", C_GREY);
            Put(15, row, "Y", C_GREY);  PutL(17, row, 3, r[1], 0, 0, ColorPct(r[1]));  Put(20, row, "%", C_GREY);
            Put(23, row, "Z", C_GREY);  PutL(25, row, 3, r[2], 0, 0, ColorPct(r[2]));  Put(28, row, "%", C_GREY);
            Put(0, row + 1, "turn until all three are 100 %", C_GREY);
        }
        break;

    case RC_CAL_FLOW:
        /* roll = left-right rocking (r[1], r[3]), pitch = forward-back (r[0], r[2]) */
        if(state == TLM_CAL_RUNNING || reason == TLM_CAL_ROCK_MORE){
            Put(0, row, "rocked", C_GREY);
            Put(7, row, "roll", C_GREY);    PutL(12, row, 3, r[1], 0, 0, ColorPct(r[1]));  Put(15, row, "%", C_GREY);
            Put(18, row, "pitch", C_GREY);  PutL(24, row, 3, r[0], 0, 0, ColorPct(r[0]));  Put(27, row, "%", C_GREY);
            Put(0, row + 1, "fit", C_GREY);
            Put(7, row + 1, "roll", C_GREY);    PutL(12, row + 1, 5, r[3] * 0.001f, 2, 0, C_WHITE);
            Put(18, row + 1, "pitch", C_GREY);  PutL(24, row + 1, 5, r[2] * 0.001f, 2, 0, C_WHITE);
            Put(30, row + 1, "want 0.85", C_GREY);
        }
        else if(done || (state == TLM_CAL_FAILED && reason != TLM_CAL_CANCELLED &&
                         reason != TLM_CAL_ARMED && reason != TLM_CAL_BUSY && reason != TLM_CAL_NO_SENSOR)){
            /* counts per radian the camera really gives (the drone started with 500) */
            Put(0, row, "c/rad", C_GREY);
            Put(7, row, "roll", C_GREY);    PutL(12, row, 5, r[1], 0, 0, C_WHITE);
            Put(18, row, "pitch", C_GREY);  PutL(24, row, 5, r[0], 0, 0, C_WHITE);
            Put(30, row, "use", C_GREY);    PutL(34, row, 5, r[4], 0, 0, done ? C_GREEN : C_GREY);
            Put(0, row + 1, "fit", C_GREY);
            Put(7, row + 1, "roll", C_GREY);    PutL(12, row + 1, 5, r[3] * 0.001f, 2, 0, C_WHITE);
            Put(18, row + 1, "pitch", C_GREY);  PutL(24, row + 1, 5, r[2] * 0.001f, 2, 0, C_WHITE);
        }
        break;

    default:
        break;
    }
}

static void PageCalib(void){
    uint8_t calFresh = CalFresh();
    uint8_t running  = CalRunning();

    /* menu: arrow, name, what it sets, status the drone reports */
    for(uint8_t i = 0; i < CAL_MENU_COUNT; i++){
        uint8_t row = (uint8_t)(ROW_CAL_MENU + i);
        uint8_t sel = (i == calSel);
        if(sel) Put(1, row, ">", C_CYAN);
        Put(3, row, calName[i], sel ? C_CYAN : C_WHITE);
        Put(12, row, calWhat[i], C_GREY);
        if(i == CAL_MENU_EXIT) continue;
        uint8_t st = calFresh ? (uint8_t)((tlm.calItemStatus >> (2U * i)) & 0x03U) : TLM_CAL_ITEM_NONE;
        switch(st){
        case TLM_CAL_ITEM_OK:      Put(35, row, "OK",   C_GREEN);  break;
        case TLM_CAL_ITEM_FAIL:    Put(35, row, "FAIL", C_RED);    break;
        case TLM_CAL_ITEM_RUNNING: Put(35, row, "RUN",  C_YELLOW); break;
        default:                   Put(35, row, "--",   C_GREY);   break;
        }
    }
    for(uint8_t c = 0; c < COLS; c++) Put(c, ROW_CAL_MENU + CAL_MENU_COUNT, "-", C_GREY);

    /* while one runs the arrow stays on it */
    uint8_t item = calSel;
    uint8_t mine = calFresh && tlm.calItem == item;

    if(item < RC_CAL_COUNT && running && mine){
        for(uint8_t k = 0; k < 3; k++) Put(0, (uint8_t)(ROW_CAL_HELP + k), calDoing[item][k], C_YELLOW);
    }
    else{
        uint8_t n = CalTipScreens(item);
        if((xTaskGetTickCount() - calTipTick) >= pdMS_TO_TICKS(CAL_TIP_MS)){
            calTipTick   = xTaskGetTickCount();
            calTipScreen = (uint8_t)((calTipScreen + 1U) % (n ? n : 1U));
        }
        if(calTipScreen >= n) calTipScreen = 0;
        for(uint8_t k = 0; k < 3; k++) Put(0, (uint8_t)(ROW_CAL_HELP + k), calTips[item][calTipScreen][k], C_WHITE);
        if(n > 1U){
            char tag[12] = " tips 1/1 ";
            tag[6] = (char)('1' + calTipScreen);
            tag[8] = (char)('0' + n);
            Put(30, ROW_CAL_MENU + CAL_MENU_COUNT, tag, C_CYAN);
        }
    }
    if(item == CAL_MENU_EXIT) return;

    Put(0, ROW_CAL_STATUS, "Status :", C_GREY);
    const char *blocked = CalBlocked();
    if(running){
        if(mine) Put(9, ROW_CAL_STATUS, "RUNNING", C_YELLOW);
        else{
            Put(9, ROW_CAL_STATUS, "WAIT:", C_GREY);
            if(tlm.calItem < RC_CAL_COUNT) Put(15, ROW_CAL_STATUS, calName[tlm.calItem], C_GREY);
            Put(24, ROW_CAL_STATUS, "IS RUNNING", C_GREY);
        }
    }
    else if(CalAsking() && calAskItem == item)     Put(9, ROW_CAL_STATUS, "STARTING...", C_YELLOW);
    else if(blocked){
        Put(9, ROW_CAL_STATUS, "BLOCKED:", C_RED);
        Put(18, ROW_CAL_STATUS, blocked, C_RED);
    }
    else if(mine && tlm.calState == TLM_CAL_DONE){
        Put(9, ROW_CAL_STATUS, "DONE - IN USE NOW", C_GREEN);
        if(item == RC_CAL_IMU){
            /* the accel offset it took includes the table's tilt: over 2 deg, check the table */
            float ax = tlm.calResult[3] * 1e-4f, ay = tlm.calResult[4] * 1e-4f;
            float tilt = atan2f(sqrtf(ax * ax + ay * ay), 9.80665f) * 57.29578f;
            Put(27, ROW_CAL_STATUS, "tilt", C_GREY);
            PutL(32, ROW_CAL_STATUS, 4, tilt, 1, 0, tilt > 2.0f ? C_YELLOW : C_GREEN);
            Put(37, ROW_CAL_STATUS, "deg", C_GREY);
        }
    }
    else if(mine && tlm.calState == TLM_CAL_FAILED){
        Put(9, ROW_CAL_STATUS, "FAILED:", C_RED);
        Put(17, ROW_CAL_STATUS, CalReasonText(tlm.calReason), C_RED);
    }
    else                                           Put(9, ROW_CAL_STATUS, "READY", C_WHITE);

    if(mine && tlm.calState == TLM_CAL_RUNNING)    CalBar(ROW_CAL_BAR, tlm.calProgress, C_YELLOW);
    else if(mine && tlm.calState == TLM_CAL_DONE)  CalBar(ROW_CAL_BAR, 100, C_GREEN);
    else                                           CalBar(ROW_CAL_BAR, 0, C_GREY);

    if(mine) CalResult(item, tlm.calState, tlm.calReason, tlm.calResult);
}

/* the footer on page 6 when no warning needs it */
static void CalHint(void){
    const char *hint;
    if(CalRunning())                 hint = "OK: CANCEL";
    else if(calSel == CAL_MENU_EXIT) hint = "PREV/NEXT: ITEM   OK: PAGE 1";
    else if(!rcValid || !(rc.lastSent.flags & RC_FLAG_CAL_MODE))
                                     hint = "VIEW ONLY: CAL SWITCH IS OFF";
    else                             hint = "PREV/NEXT: ITEM   OK: START";
    Put(0, ROW_FOOTER, hint, C_GREY);
}

/* the button on page 6; 1 = leave the page */
static uint8_t CalButton(BtnEvent_t ev){
    Snapshot();                             /* decide on the drone's state now, not at the last refresh */
    if(ev == BTN_NEXT || ev == BTN_PREV){
        if(!CalRunning()){                  /* while one runs the arrow stays on it */
            calSel       = (ev == BTN_NEXT) ? (uint8_t)((calSel + 1U) % CAL_MENU_COUNT)
                                            : (uint8_t)((calSel + CAL_MENU_COUNT - 1U) % CAL_MENU_COUNT);
            calTipScreen = 0;
            calTipTick   = xTaskGetTickCount();
        }
        return 0;
    }
    /* OK */
    if(CalRunning()){
        RCTx_CalCancel();
        return 0;
    }
    if(calSel == CAL_MENU_EXIT){
        calSel = 0;
        return 1;
    }
    if(CalBlocked() != NULL) return 0;      /* the status line says why */
    RCTx_CalStart(calSel);
    calAsked     = 1;
    calAskItem   = calSel;
    calAskTick   = xTaskGetTickCount();
    calAskFrames = tlm.calFrames;
    return 0;
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
    case PAGE_CALIB:    PageCalib();    break;
    default:            break;
    }
    if(!Footer(page) && page == PAGE_CALIB) CalHint();
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

/* PREV, NEXT, OK: each debounced on its own; an event goes out when a press
 * is confirmed (20 ms down), so the page reacts at once. */
static void ButtonTask(void *argument){
    (void)argument;
    static GPIO_TypeDef *const port[BTN_COUNT] = { BTN_PREV_PORT, BTN_NEXT_PORT, BTN_OK_PORT };
    static const uint16_t      pin[BTN_COUNT]  = { BTN_PREV_PIN,  BTN_NEXT_PIN,  BTN_OK_PIN  };

    /* the pins are inputs with pull-up from CubeMX (gpio.c) */
    uint8_t    stable[BTN_COUNT]   = {0};
    uint8_t    debounce[BTN_COUNT] = {0};
    TickType_t wake = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(BTN_TICK_MS));

        for(uint8_t b = 0; b < BTN_COUNT; b++){
            uint8_t pressed = (HAL_GPIO_ReadPin(port[b], pin[b]) == GPIO_PIN_RESET);
            if(pressed == stable[b]){
                debounce[b] = 0;
                continue;
            }
            if(++debounce[b] < BTN_DEBOUNCE_TICKS) continue;
            stable[b]   = pressed;
            debounce[b] = 0;
            if(pressed){
                BtnEvent_t ev = (BtnEvent_t)b;
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
            if(page == PAGE_CALIB){
                if(CalButton(ev)) page = PAGE_STATUS;
            }
            else if(ev == BTN_NEXT) page = (Page_t)((page + 1) % PAGE_COUNT);
            else if(ev == BTN_PREV) page = (Page_t)((page + PAGE_COUNT - 1) % PAGE_COUNT);
            else                    page = PAGE_STATUS;         /* OK: home */
        }

        /* calibration switch: on opens page 6, off goes back to the page shown before
         * it was switched on (the drone cancels a running calibration). Nothing else
         * changes the page by itself. */
        {
            static uint8_t calSwLast     = 0;
            static Page_t  pageBeforeCal = PAGE_STATUS;
            RCTx_Status_t now;
            uint8_t calSw = RCTx_GetStatus(&now) && (now.lastSent.flags & RC_FLAG_CAL_MODE);
            if(calSw && !calSwLast){
                pageBeforeCal = page;           /* already on page 6: switching off stays there */
                page          = PAGE_CALIB;
                calSel = 0;
            }
            else if(!calSw && calSwLast && page == PAGE_CALIB){
                page = pageBeforeCal;
            }
            calSwLast = calSw;
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
