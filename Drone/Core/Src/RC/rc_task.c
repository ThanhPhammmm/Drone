#include "rc_task.h"
#include "nrf24.h"
#include "rc_protocol.h"
#include "rc_topic.h"
#include "arm.h"
#include "attitude_setpoint_topic.h"
#include "altitude_setpoint_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stm32f4xx.h>

/* The loop wakes on the radio IRQ but also on a timeout, because failsafe is
 * exactly the case where no interrupt is ever going to arrive. */
#define RC_TICK_MS                  20
#define RC_TIMEOUT_MS               300     /* no valid frame -> link lost */

#define RC_ARM_THROTTLE_MAX         0.05f   /* must be at idle to arm */
#define RC_FAILSAFE_DESCENT_MS      (-1.0f) /* m/s */
#define RC_FAILSAFE_THRUST_SCALE    0.90f   /* blind descent if no baro */

#define RC_MAX_TILT_RAD             0.35f   /* ~20 deg at full stick */
#define RC_MAX_YAW_RATE             3.0f    /* rad/s */
#define RC_MAX_CLIMB_RATE           2.0f    /* m/s at full throttle deflection */
#define RC_THROTTLE_DEADBAND        0.05f

static RC_Data_t rc;

static float RC_Norm(int16_t v){
    float f = (float)v / (float)RC_CHANNEL_MAX;
    if(f >  1.0f) f =  1.0f;
    if(f < -1.0f) f = -1.0f;
    return f;
}

void RCTask(void *argument){
    NRF24_SetTaskHandle(xTaskGetCurrentTaskHandle());

    uint8_t radioOk = (NRF24_Init() == NRF24_OK);

    RC_Packet_t pkt;
    RC_Packet_t latest    = {0};
    uint8_t     haveFrame = 0;
    uint8_t     lastSeq   = 0;
    TickType_t  lastFrame = xTaskGetTickCount();

    float lastThrottle = 0.0f;

    while(1){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RC_TICK_MS));

        /* Drain the whole 3-deep FIFO and keep only the newest frame -- acting
         * on a queued stale frame would fly the aircraft on old stick data. */
        if(radioOk){
            while(NRF24_ReadPacket(&pkt) == NRF24_OK){
                if(pkt.magic != RC_PACKET_MAGIC) continue;
                if(haveFrame && pkt.seq == lastSeq) continue;   /* duplicate */

                lastSeq   = pkt.seq;
                latest    = pkt;
                haveFrame = 1;
                lastFrame = xTaskGetTickCount();
            }
        }

        TickType_t age    = xTaskGetTickCount() - lastFrame;
        uint8_t    linkOk = radioOk && haveFrame && (age < pdMS_TO_TICKS(RC_TIMEOUT_MS));

        if(linkOk){
            rc.roll     = RC_Norm(latest.roll);
            rc.pitch    = RC_Norm(latest.pitch);
            rc.yaw      = RC_Norm(latest.yaw);
            rc.throttle = (float)latest.throttle / (float)RC_THROTTLE_MAX;
            if(rc.throttle > 1.0f) rc.throttle = 1.0f;
            lastThrottle = rc.throttle;

            /* RATE mode needs the attitude controller to pass sticks straight
             * through as rates, which it cannot do yet -- fall back to ANGLE
             * rather than silently flying something the pilot did not pick. */
            rc.mode = (latest.mode == RC_MODE_ALT_HOLD) ? RC_MODE_ALT_HOLD
                                                        : RC_MODE_ANGLE;
        }else{
            rc.roll = rc.pitch = rc.yaw = 0.0f;
            /* rc.throttle deliberately retains its last value -- the blind
             * descent below needs it. */
        }

        /* One authority for whether motors may spin. This task only reports
         * inputs; every transition rule lives in arm.c. */
        uint8_t armRequest   = linkOk && (latest.flags & RC_FLAG_ARM);
        uint8_t throttleIdle = (rc.throttle < RC_ARM_THROTTLE_MAX);

        Arm_Update(armRequest, throttleIdle, linkOk,
                   (uint32_t)(age * portTICK_PERIOD_MS));

        AttitudeSetpoint_Data_t attSp = {0};
        AltitudeSetpoint_Data_t altSp = {0};

        if(linkOk){
            attSp.roll    = rc.roll  * RC_MAX_TILT_RAD;
            attSp.pitch   = rc.pitch * RC_MAX_TILT_RAD;
            attSp.yawRate = rc.yaw   * RC_MAX_YAW_RATE;

            if(rc.mode == RC_MODE_ALT_HOLD){
                /* stick about centre commands climb rate; centred = hold */
                float t = (rc.throttle - 0.5f) * 2.0f;
                if(t > -RC_THROTTLE_DEADBAND && t < RC_THROTTLE_DEADBAND) t = 0.0f;

                altSp.climbRate    = t * RC_MAX_CLIMB_RATE;
                altSp.holdEnabled  = Arm_MotorsAllowed();
                altSp.manualThrust = rc.throttle;
            }
            else{
                altSp.climbRate    = 0.0f;
                altSp.holdEnabled  = 0;
                altSp.manualThrust = rc.throttle;
            }
        }
        else{
            /* Failsafe: level the aircraft, stop yawing, and descend. */
            attSp.roll = attSp.pitch = attSp.yawRate = 0.0f;

            /* Altitude hold handles the descent when the estimate is good.
             * If it is not, the controller falls back to manualThrust, so bias
             * that below the last known throttle for a blind sink. This is a
             * hard landing, not a graceful one -- the disarm timeout in arm.c
             * is the real backstop. */
            altSp.climbRate    = RC_FAILSAFE_DESCENT_MS;
            altSp.holdEnabled  = Arm_MotorsAllowed();
            altSp.manualThrust = lastThrottle * RC_FAILSAFE_THRUST_SCALE;
        }

        if(arm_state == DISARMED){
            altSp.holdEnabled  = 0;
            altSp.manualThrust = 0.0f;
        }

        rc.linkOk       = linkOk;
        rc.timestamp_us = DWT->CYCCNT / (SystemCoreClock / 1000000U);

        attSp.timestamp_us = rc.timestamp_us;
        altSp.timestamp_us = rc.timestamp_us;

        RCTopic_Publish(&rc);
        AttitudeSetpointTopic_Publish(&attSp);
        AltitudeSetpointTopic_Publish(&altSp);
    }
}
