#include "motor_output.h"
#include "tim.h"
#include "arm.h"

#define MOTOR_TIM      htim4
#define MOTOR_PWM_MIN  1000.0f   /* µs = throttle 0 / disarmed */
#define MOTOR_PWM_MAX  2000.0f   /* µs = throttle 1 */
#define MOTOR_IDLE     0.0f     /* spin-min when ARMED (0..1); set 0.0f for very first test */

static const uint32_t motorChannel[MOTOR_COUNT] = {
    TIM_CHANNEL_1, TIM_CHANNEL_2, TIM_CHANNEL_3, TIM_CHANNEL_4
};

static const float mix[MOTOR_COUNT][4] = {
    /* M1 front-right */ { -1.0f, +1.0f, +1.0f, 1.0f },
    /* M2 rear-right  */ { -1.0f, -1.0f, -1.0f, 1.0f },
    /* M3 rear-left   */ { +1.0f, -1.0f, +1.0f, 1.0f },
    /* M4 front-left  */ { +1.0f, +1.0f, -1.0f, 1.0f },
};

static inline float clampf(float v, float lo, float hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

static void motor_write(uint8_t i, float value01){   /* value01 in [0,1] */
    uint16_t ccr = (uint16_t)(MOTOR_PWM_MIN + value01 * (MOTOR_PWM_MAX - MOTOR_PWM_MIN));
    __HAL_TIM_SET_COMPARE(&MOTOR_TIM, motorChannel[i], ccr);
}

void MotorOutput_Init(void){
    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        HAL_TIM_PWM_Start(&MOTOR_TIM, motorChannel[i]);
        motor_write(i, 0.0f);      /* 1000 µs -> ESCs arm at power-up */
    }
}

void MotorOutput_Update(float roll, float pitch, float yaw, float throttle){
    if(arm_state != ARMED){
        for(uint8_t i = 0; i < MOTOR_COUNT; i++) motor_write(i, 0.0f);
        return;
    }

    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        float out = throttle * mix[i][3]
                  + roll     * mix[i][0]
                  + pitch    * mix[i][1]
                  + yaw      * mix[i][2];
        out = clampf(out, MOTOR_IDLE, 1.0f);
        motor_write(i, out);
    }
}
