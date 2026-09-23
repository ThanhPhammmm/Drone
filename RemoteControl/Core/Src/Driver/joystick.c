#include "joystick.h"
#include "FreeRTOS.h"
#include "task.h"
#include "adc.h"


/* ==========================================================
 * RC JOYSTICK DEADBAND CONFIGURATION
 * ==========================================================
 * Concept: A tolerance zone (in raw ADC) around the stick center.
 * Any value inside this zone is forced to absolute 0.
 *
 * Why it's required:
 * 1. Mechanical error: Stick springs never return to exact center.
 * 2. ADC Jitter: Electrical noise causes constant signal fluctuation.
 * ========================================================== */

#define STICK_DEADBAND       100        /* raw ADC counts around center to ignore */
#define ADC_RESOLUTION_MAX   4095      /* 12-bit */

static volatile uint16_t adcRaw[JOY_NUM_CHANNELS];
static uint16_t stickCenter[JOY_NUM_CHANNELS];   /* roll/pitch/yaw only */

static uint16_t JOY_SampleAveraged(uint8_t ch, uint8_t samples){
    uint32_t acc = 0;
    for(uint8_t i = 0; i < samples; i++){
        acc += adcRaw[ch];
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return (uint16_t)(acc / samples);
}

void JOY_DMA_Callback(ADC_HandleTypeDef *hadc){
    BaseType_t hpw = pdFALSE;
    portYIELD_FROM_ISR(hpw);
}

void JOY_Init(void){
    HAL_ADC_Start_DMA(&JOY_ADC, (uint32_t *)adcRaw, JOY_NUM_CHANNELS);
    vTaskDelay(pdMS_TO_TICKS(20));  /* let the DMA buffer fill at least once */

    stickCenter[JOY_CH_ROLL]  = JOY_SampleAveraged(JOY_CH_ROLL,  16);
    stickCenter[JOY_CH_PITCH] = JOY_SampleAveraged(JOY_CH_PITCH, 16);
    stickCenter[JOY_CH_YAW]   = JOY_SampleAveraged(JOY_CH_YAW,   16);
    /* Throttle deliberately has no center calibration -- it's mapped as an
     * absolute 0..max stick, not a centered one. */
}

static int16_t JOY_CenteredChannel(uint8_t ch, uint8_t invert){
    int32_t raw = (int32_t)adcRaw[ch] - (int32_t)stickCenter[ch];

    if(raw > -STICK_DEADBAND && raw < STICK_DEADBAND) raw = 0;

    /* Half-range varies slightly high/low side of center on a real pot, so
     * scale against whichever half the sample currently sits in. */
    int32_t halfRange = (raw >= 0) ? (ADC_RESOLUTION_MAX - stickCenter[ch])
                                    : stickCenter[ch];
    if(halfRange <= 0) halfRange = 1;

    int32_t scaled = (raw * RC_CHANNEL_MAX) / halfRange;
    if(scaled >  RC_CHANNEL_MAX) scaled =  RC_CHANNEL_MAX;
    if(scaled < -RC_CHANNEL_MAX) scaled = -RC_CHANNEL_MAX;

    return invert ? (int16_t)(-scaled) : (int16_t)scaled;
}

void JOY_Read(RC_Packet_t *pkt){
    pkt->roll  = -JOY_CenteredChannel(JOY_CH_ROLL,  0);
    pkt->pitch = -JOY_CenteredChannel(JOY_CH_PITCH, 0);
    pkt->yaw   = -JOY_CenteredChannel(JOY_CH_YAW,   0);

    int32_t thr = ((int32_t)adcRaw[JOY_CH_THROTTLE] * RC_THROTTLE_MAX) / ADC_RESOLUTION_MAX;
    if(thr < 0) thr = 0;
    if(thr > RC_THROTTLE_MAX) thr = RC_THROTTLE_MAX;
    pkt->throttle = (uint16_t)thr;

    /* Switch closed to GND (pin reads LOW) = armed. */
    pkt->flags = ((HAL_GPIO_ReadPin(ARM_SW_PORT, ARM_SW_PIN) == GPIO_PIN_RESET) ? RC_FLAG_ARM : 0);
}
