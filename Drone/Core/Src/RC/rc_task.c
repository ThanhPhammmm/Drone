#include "rc_task.h"

/* The loop wakes on the radio IRQ but also on a timeout, because failsafe is
 * exactly the case where no interrupt is ever going to arrive. */
#define RC_TICK_MS                  20
#define RC_TIMEOUT_MS               500     /* no valid frame -> link lost */

#define RC_ARM_THROTTLE_MAX         0.02f   /* must be at idle to arm */
#define RC_FAILSAFE_DESCENT_MS      (-1.0f) /* m/s */
#define RC_FAILSAFE_THRUST_SCALE    0.90f   /* blind descent if no baro */

#define RC_MAX_TILT_RAD             0.35f   /* ~20 deg at full stick */
#define RC_MAX_YAW_RATE             3.0f    /* rad/s */

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
    uint16_t    rcLost    = 0;	/* seq gaps, for telemetry */
    uint8_t     tlm[32];

    float lastThrottle = 0.0f;

    while(1){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RC_TICK_MS));
        uint8_t gotPacket = 0;
        if(radioOk){
            for(uint8_t n = 0; n < 4; n++){         /* FIFO is 3 deep; bounded even if SPI fails */
                NRF24_Status_t st = NRF24_ReadPacket(&pkt);
                if(st == NRF24_EMPTY) break;
                if(st != NRF24_OK) continue;
                if(pkt.magic != RC_PACKET_MAGIC) continue;
                if(haveFrame && pkt.seq == lastSeq) continue;   /* duplicate */

                if(haveFrame) rcLost += (uint8_t)(pkt.seq - lastSeq - 1U);
                lastSeq   = pkt.seq;
                latest    = pkt;
                haveFrame = 1;
                gotPacket = 1;
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
        }
        else{
            rc.roll = rc.pitch = rc.yaw = 0.0f;
            /* rc.throttle deliberately retains its last value -- the blind
             * descent below needs it. */
        }

        Altitude_Data_t altNow = {0};
        uint8_t altValid = (AltitudeTopic_Copy(&altNow, 0) == pdPASS) && altNow.valid;

        /* Flight mode actually flown. On link loss: ALT_HOLD descent if the
         * altitude estimate is good, otherwise ANGLE with a blind thrust cut. */
        FlightMode_t mode;
        if(linkOk) mode = FlightMode_Resolve(latest.mode, altValid);
        else       mode = altValid ? FLIGHT_MODE_ALT_HOLD : FLIGHT_MODE_ANGLE;
        rc.mode = (uint8_t)mode;
        FlightMode_SetActive(mode);

        Thrust_Data_t thrustNow = {0};
        if(ThrustTopic_Copy(&thrustNow, 0) != pdPASS) thrustNow.thrust = 0.0f;

        Arm_Input_t armIn;
        armIn.armSwitch      = (latest.flags & RC_FLAG_ARM) ? 1U : 0U;
        armIn.sensorsReady   = Calib_ReadyToArm();;
        armIn.linkOk         = linkOk;
        armIn.linkLostMs     = (uint32_t)(age * portTICK_PERIOD_MS);
        armIn.throttleLow    = (rc.throttle < RC_ARM_THROTTLE_MAX);
        /* take-off/landing come from the pilot's stick, not from the thrust the
         * controller happens to output (in ALT_HOLD those are unrelated) */
        armIn.takeoffRequest = linkOk && FlightMode_TakeoffRequested(mode, rc.throttle);
        armIn.descendRequest = linkOk && FlightMode_DescendRequested(mode, rc.throttle);
        armIn.thrust         = thrustNow.thrust;
        armIn.altitude       = altNow.altitude;
        armIn.verticalSpeed  = altNow.verticalSpeed;
        armIn.altitudeValid  = altValid;

        Arm_Update(&armIn);

        AttitudeSetpoint_Data_t attSp = {0};
        AltitudeSetpoint_Data_t altSp = {0};
        altSp.mode = (uint8_t)mode;

        if(linkOk){
            attSp.roll    = rc.roll  * RC_MAX_TILT_RAD;
            attSp.pitch   = rc.pitch * RC_MAX_TILT_RAD;
            attSp.yawRate = rc.yaw   * RC_MAX_YAW_RATE;

            switch(mode){
            case FLIGHT_MODE_POS_HOLD:
                /* TODO: position controller publishes roll/pitch here instead of the
                 * sticks (see flight_mode.c). Vertical axis is ALT_HOLD. */
                /* fall through */
            case FLIGHT_MODE_ALT_HOLD:
                /* stick about centre commands climb rate; centred = hold */
                altSp.climbRate    = FlightMode_ClimbRate(rc.throttle);
                altSp.manualThrust = rc.throttle;     /* used only if the estimate drops out */
                break;

            case FLIGHT_MODE_ANGLE:
            default:
                altSp.climbRate    = 0.0f;
                altSp.manualThrust = rc.throttle;
                break;
            }
        }
        else{
            /* Failsafe: level the aircraft, stop yawing, and descend. */
            attSp.roll = attSp.pitch = attSp.yawRate = 0.0f;

            /* ALT_HOLD handles the descent when the estimate is good. If it is
             * not, mode is ANGLE and manualThrust is flown directly, so bias it
             * below the last known throttle for a blind sink. This is a hard
             * landing, not a graceful one -- the disarm timeout in arm.c is the
             * real backstop. */
            altSp.climbRate    = RC_FAILSAFE_DESCENT_MS;
            altSp.manualThrust = lastThrottle * RC_FAILSAFE_THRUST_SCALE;
        }

        if(!Arm_MotorsAllowed()){
            altSp.climbRate    = 0.0f;
            altSp.manualThrust = 0.0f;
        }

        rc.linkOk       = linkOk;
        rc.timestamp_us = Time_Us();;

        attSp.timestamp_us = rc.timestamp_us;
        altSp.timestamp_us = rc.timestamp_us;

        RCTopic_Publish(&rc);
        AttitudeSetpointTopic_Publish(&attSp);
        AltitudeSetpointTopic_Publish(&altSp);
        if(gotPacket){
            uint8_t len = Telemetry_Build(tlm, lastSeq, rcLost);
            NRF24_WriteAckPayload(tlm, len);
        }
    }
}
