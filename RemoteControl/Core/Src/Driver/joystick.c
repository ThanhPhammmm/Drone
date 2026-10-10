#include "joystick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"
#include <math.h>

#define ADC_RESOLUTION_MAX      4095        /* 12-bit */
#define JOY_SCANS               32          /* ADC scans averaged per read, ~25 us each */

#define JOY_CENTER_SETTLE_MS    100         /* let the pots and the ADC settle first */
#define JOY_CENTER_SAMPLES      32
#define JOY_CENTER_SAMPLE_MS    5

#define JOY_DEADBAND            100.0f       /* raw counts around centre that read 0 (~2% of travel) */
#define JOY_DEFAULT_HALF_TRAVEL 1400.0f     /* raw counts for full deflection until the stick has gone further */

#define JOY_TRIM_WINDOW         (2.0f * JOY_DEADBAND)
#define JOY_TRIM_STILL          12.0f       /* raw distance from the smoothed value below this = not being touched */
#define JOY_TRIM_SMOOTH_MS      100.0f      /* smoothing used for that stillness test */
#define JOY_TRIM_HOLD_MS        1000U       /* still this long before the centre starts to follow */
#define JOY_TRIM_TAU_MS         2000.0f     /* time constant of the centre following a resting stick */

#define JOY_THR_END_DEADBAND    100.0f       /* raw counts at each throttle end that read exactly 0 / 1000 */

typedef struct{
    uint8_t  ch;
    float    center;
    float    lo;            /* lowest raw value this stick has reached */
    float    hi;            /* highest raw value this stick has reached */
    float    smooth;        /* low-passed raw, for the stillness test */
    uint32_t stillMs;
} JOY_Axis_t;

static volatile uint16_t adcRing[JOY_SCANS * JOY_NUM_CHANNELS];

static JOY_Axis_t axisRoll  = { .ch = JOY_CH_ROLL  };
static JOY_Axis_t axisPitch = { .ch = JOY_CH_PITCH };
static JOY_Axis_t axisYaw   = { .ch = JOY_CH_YAW   };

static TickType_t lastReadTick;
static volatile uint16_t throttleRaw;       /* last averaged ADC count, for the stick check */

static float JOY_Average(uint8_t ch){
    uint32_t acc = 0;
    for(uint16_t s = 0; s < JOY_SCANS; s++){
        acc += adcRing[s * JOY_NUM_CHANNELS + ch];
    }
    return (float)acc / (float)JOY_SCANS;
}

static void JOY_AxisInit(JOY_Axis_t *a, float center){
    a->center  = center;
    a->lo      = center - JOY_DEFAULT_HALF_TRAVEL;
    a->hi      = center + JOY_DEFAULT_HALF_TRAVEL;
    if(a->lo < 0.0f)                      a->lo = 0.0f;
    if(a->hi > (float)ADC_RESOLUTION_MAX) a->hi = (float)ADC_RESOLUTION_MAX;
    a->smooth  = center;
    a->stillMs = 0;
}

static int16_t JOY_Round(float v){
    return (int16_t)(v >= 0.0f ? v + 0.5f : v - 0.5f);
}

static int16_t JOY_AxisRead(JOY_Axis_t *a, uint32_t dtMs){
    float raw = JOY_Average(a->ch);

    if(raw < a->lo) a->lo = raw;
    if(raw > a->hi) a->hi = raw;

    float off = raw - a->center;

    a->smooth += (raw - a->smooth) * ((float)dtMs / (JOY_TRIM_SMOOTH_MS + (float)dtMs));

    if(fabsf(off) < JOY_TRIM_WINDOW && fabsf(raw - a->smooth) < JOY_TRIM_STILL){
        if(a->stillMs < JOY_TRIM_HOLD_MS){
            a->stillMs += dtMs;
        }
        else{
            a->center += off * ((float)dtMs / JOY_TRIM_TAU_MS);
            off = raw - a->center;
        }
    }
    else{
        a->stillMs = 0;
    }

    if(off > -JOY_DEADBAND && off < JOY_DEADBAND) return 0;

    float travel, x;
    if(off > 0.0f){
        travel = a->hi - a->center - JOY_DEADBAND;
        x      = off - JOY_DEADBAND;
    }
    else{
        travel = a->center - a->lo - JOY_DEADBAND;
        x      = off + JOY_DEADBAND;
    }
    if(travel < 1.0f) travel = 1.0f;

    float v = x / travel * (float)RC_CHANNEL_MAX;
    if(v >  (float)RC_CHANNEL_MAX) v =  (float)RC_CHANNEL_MAX;
    if(v < -(float)RC_CHANNEL_MAX) v = -(float)RC_CHANNEL_MAX;
    return JOY_Round(v);
}

static uint16_t JOY_ThrottleRead(void){
    float raw  = JOY_Average(JOY_CH_THROTTLE);
    throttleRaw = (uint16_t)raw;
    float span = (float)ADC_RESOLUTION_MAX - 2.0f * JOY_THR_END_DEADBAND;

    float v = (raw - JOY_THR_END_DEADBAND) / span * (float)RC_THROTTLE_MAX;
    if(v < 0.0f)                    v = 0.0f;
    if(v > (float)RC_THROTTLE_MAX)  v = (float)RC_THROTTLE_MAX;
    return (uint16_t)JOY_Round(v);
}

uint16_t JOY_ThrottleRaw(void){
    return throttleRaw;
}

void JOY_DMA_Callback(ADC_HandleTypeDef *hadc){
    BaseType_t hpw = pdFALSE;
    portYIELD_FROM_ISR(hpw);
}

void JOY_Init(void){
    HAL_ADC_Start_DMA(&JOY_ADC, (uint32_t *)adcRing, JOY_SCANS * JOY_NUM_CHANNELS);
    vTaskDelay(pdMS_TO_TICKS(JOY_CENTER_SETTLE_MS));

    /* Sticks must be released at power-up. Throttle has no centre: it is an
     * absolute 0..max stick, not a centred one. */
    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f;
    for(uint8_t i = 0; i < JOY_CENTER_SAMPLES; i++){
        roll  += JOY_Average(JOY_CH_ROLL);
        pitch += JOY_Average(JOY_CH_PITCH);
        yaw   += JOY_Average(JOY_CH_YAW);
        vTaskDelay(pdMS_TO_TICKS(JOY_CENTER_SAMPLE_MS));
    }
    JOY_AxisInit(&axisRoll,  roll  / (float)JOY_CENTER_SAMPLES);
    JOY_AxisInit(&axisPitch, pitch / (float)JOY_CENTER_SAMPLES);
    JOY_AxisInit(&axisYaw,   yaw   / (float)JOY_CENTER_SAMPLES);

    lastReadTick = xTaskGetTickCount();
}

void JOY_Read(RC_Packet_t *pkt){
    TickType_t now  = xTaskGetTickCount();
    uint32_t   dtMs = (uint32_t)(now - lastReadTick) * portTICK_PERIOD_MS;
    if(dtMs > 100U) dtMs = 100U;
    lastReadTick = now;

    pkt->roll     = -JOY_AxisRead(&axisRoll,  dtMs);
    pkt->pitch    = -JOY_AxisRead(&axisPitch, dtMs);
    pkt->yaw      = -JOY_AxisRead(&axisYaw,   dtMs);
    pkt->throttle = JOY_ThrottleRead();

    /* Switch closed to GND (pin reads LOW) = armed. */
    pkt->flags = ((HAL_GPIO_ReadPin(ARM_SW_PORT, ARM_SW_PIN) == GPIO_PIN_RESET) ? RC_FLAG_ARM : 0);
}
