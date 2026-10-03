#include "actuator.h"
#include "tim.h"
#include "arm.h"
#include "debug.h"

#define MOTOR_TIM      htim4
#define MOTOR_PWM_MIN  1000.0f   /* µs = throttle 0 / disarmed */
#define MOTOR_PWM_MAX  2000.0f   /* µs = throttle 1 */
#define MOTOR_IDLE     0.05f     /* spin-min while motors are allowed to move (0..1) */
#define MOTOR_MAX      1.0f

#define MOTOR_DESAT_MIN_THROTTLE  0.05f

static const uint32_t motorChannel[MOTOR_COUNT] = {
	TIM_CHANNEL_4, TIM_CHANNEL_2, TIM_CHANNEL_3,TIM_CHANNEL_1
};

static const float mix[MOTOR_COUNT][4] = {
    /* M1 front-right CCW*/ { -1.0f, +1.0f, +1.0f, 1.0f },
    /* M2 rear-right  CW*/ { -1.0f, -1.0f, -1.0f, 1.0f },
    /* M3 rear-left   CCW*/ { +1.0f, -1.0f, +1.0f, 1.0f },
    /* M4 front-left  CW*/ { +1.0f, +1.0f, -1.0f, 1.0f },
};

static inline float clampf(float v, float lo, float hi){
    return v < lo ? lo : (v > hi ? hi : v);
}

static void motor_write(uint8_t i, float value01){   /* value01 in [0,1] */
    value01 = clampf(value01, 0.0f, 1.0f);
    uint32_t ccr = (uint32_t)(MOTOR_PWM_MIN + value01 * (MOTOR_PWM_MAX - MOTOR_PWM_MIN) + 0.5f);
     __HAL_TIM_SET_COMPARE(&MOTOR_TIM, motorChannel[i], ccr);
}

void MotorOutput_Init(void){
    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        HAL_TIM_PWM_Start(&MOTOR_TIM, motorChannel[i]);
        motor_write(i, 0.0f);      /* 1000 µs -> ESCs arm at power-up */
    }
}

void MotorOutput_Update(float roll, float pitch, float yaw, float throttle, MotorSaturation_t *sat){
	if(sat){
		sat->rollPitchSaturated		= 0;
		sat->yawSaturated			= 0;
		sat->throttleSaturatedHigh	= 0;
		sat->throttleSaturatedLow	= 0;
	}

	if(!Arm_MotorsAllowed()){
		/* ARMED_GROUND spins the props at idle, everything else keeps them stopped */
		float idle = Arm_GroundIdle() ? MOTOR_IDLE : 0.0f;
        for(uint8_t i = 0; i < MOTOR_COUNT; i++) motor_write(i, idle);
        return;
    }

    const float available = MOTOR_MAX - MOTOR_IDLE;

    /* Roll/pitch first: they keep the aircraft upright. */
    float rp[MOTOR_COUNT];
    float lo = 0.0f, hi = 0.0f;

    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        rp[i] = roll  * mix[i][0]
              + pitch * mix[i][1];

        if(i == 0 || rp[i] < lo) lo = rp[i];
        if(i == 0 || rp[i] > hi) hi = rp[i];
    }

    float scale = 1.0f;
    if((hi - lo) > available){
        scale = available / (hi - lo);
        lo *= scale;
        hi *= scale;
        if(sat) sat->rollPitchSaturated = 1;
    }

    /* Yaw only gets the room roll/pitch left, so a large yaw demand can never
     * scale roll/pitch down. The yaw mix is +-1, so |yaw| <= headroom/2 cannot
     * push the spread past `available`. */
    float yawMax = 0.5f * (available - (hi - lo));
    if(yaw > yawMax || yaw < -yawMax){
        yaw = clampf(yaw, -yawMax, yawMax);
        if(sat) sat->yawSaturated = 1;
    }

    float axis[MOTOR_COUNT];
    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        axis[i] = rp[i] * scale + yaw * mix[i][2];

        if(i == 0 || axis[i] < lo) lo = axis[i];
        if(i == 0 || axis[i] > hi) hi = axis[i];
    }

    /* Air-mode: lift the collective so a torque demand can still be met at low
     * throttle. Only once airborne -- on the ground it lets the attitude loop
     * raise one side of the frame and flip it over its own legs. */
    float thrMin = (Arm_IsAirborne() && throttle > MOTOR_DESAT_MIN_THROTTLE) ? (MOTOR_IDLE - lo) : MOTOR_IDLE;
    float thrMax = MOTOR_MAX - hi;

    float thr = clampf(throttle, thrMin, thrMax);

    if(sat){
    	if(throttle > thrMax) sat->throttleSaturatedHigh = 1;
    	if(throttle < thrMin) sat->throttleSaturatedLow  = 1;
	}

    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        float out = clampf(thr + axis[i], MOTOR_IDLE, MOTOR_MAX);
        motor_write(i, out);
    }
}

void ESC_Calibrate(void){
    //HAL_Delay(3000);

    for(uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_write(i, 1.0f);   // 2000us
    }

    //HAL_Delay(3000);

    for(uint8_t i = 0; i < MOTOR_COUNT; i++)
    {
        motor_write(i, 0.0f);   // 1000us
    }

    //HAL_Delay(3000);
}

void MotorOutput_GetPulsesUs(float us[MOTOR_COUNT]){
    for(uint8_t i = 0; i < MOTOR_COUNT; i++){
        us[i] = (float)__HAL_TIM_GET_COMPARE(&MOTOR_TIM, motorChannel[i]);   /* 1 tick = 1 us */
    }
}
