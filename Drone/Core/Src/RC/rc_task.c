#include "rc_task.h"

/* The loop wakes on the radio IRQ but also on a timeout, because failsafe is
 * exactly the case where no interrupt is ever going to arrive. */
#define RC_TICK_MS                  20
#define RC_TIMEOUT_MS               500     /* no valid frame -> link lost */

#define RC_ARM_THROTTLE_MAX         0.02f   /* must be at idle to arm */
#define RC_FAILSAFE_DESCENT_MS      (-1.0f) /* m/s */
#define RC_FAILSAFE_THRUST_SCALE    0.90f   /* blind descent if no baro */

#define RC_MAX_TILT_RAD             0.52f   /* ~30 deg at full stick, check attitude_controller_task.c accordingly */
#define RC_HOLD_RELEASE_LOW         0.10f   /* ALT_HOLD waiting for a centred stick: this low = descend */
#define RC_MAX_YAW_RATE             3.0f    /* rad/s */
#define RC_MODE_REENTRY_MS          1000U   /* an estimate that dropped out must be valid this long before
                                             * its hold mode is entered again (no flicking between modes) */
#define RC_RADIO_CHECK_MS           150U    /* no frame this long: check the radio this often, set it up again if needed.
                                             * A radio that lost its settings in flight is back within ~0.4 s
                                             * (this + the 0.2 s its init takes), before RC_TIMEOUT_MS calls the
                                             * failsafe. At 1000 it was 1.2 s, and a descent every time */

static RC_Data_t rc;

static uint8_t prevHoldEngaged = 0;
static uint8_t waitStickCentre = 0;

static float RC_Norm(int16_t v){
    float f = (float)v / (float)RC_CHANNEL_MAX;
    if(f >  1.0f) f =  1.0f;
    if(f < -1.0f) f = -1.0f;
    return f;
}

static RC_ValidTimer_t altTimer;
static RC_ValidTimer_t posTimer;
static uint8_t         posDropped = 0;   /* the position estimate went away during this flight */

static uint8_t RC_Usable(RC_ValidTimer_t *t, uint8_t valid, uint8_t inUse, TickType_t now){
    if(!valid){
        t->wasValid = 0;
        return 0;
    }
    if(!t->wasValid){
        t->wasValid   = 1;
        t->validSince = now;
    }
    return inUse || (TickType_t)(now - t->validSince) >= pdMS_TO_TICKS(RC_MODE_REENTRY_MS);
}

static float RC_HoldThrottle(uint8_t holdMode, uint8_t linkOk, uint8_t *holdEngaged){
    uint8_t engaged = holdMode && Arm_IsAirborne();
    if(!engaged || !linkOk)   waitStickCentre = 0;
    else if(!prevHoldEngaged) waitStickCentre = 1;
    if(waitStickCentre && (FlightMode_ClimbRate(rc.throttle) == 0.0f || rc.throttle < RC_HOLD_RELEASE_LOW)){
        waitStickCentre = 0;
    }
    prevHoldEngaged = engaged;
    *holdEngaged    = engaged;
    return waitStickCentre ? 0.5f : rc.throttle;  /* 0.5f(center stick) is a pretended value for being stable in the air */
}

void RCTask(void *argument){
    NRF24_SetTaskHandle(xTaskGetCurrentTaskHandle());

    uint8_t radioOk = (NRF24_Init() == NRF24_OK);
    TickType_t lastRadioCheck  = xTaskGetTickCount();

    RC_Packet_t pkt;
    RC_Packet_t latest    = {0};
    uint8_t     haveFrame = 0;
    uint8_t     lastSeq   = 0;
    TickType_t  lastFrame = xTaskGetTickCount();
    uint16_t    rcLost    = 0;	/* seq gaps, for telemetry */
    uint8_t     tlm[32];

    float lastThrottle = 0.0f;
    float cosTilt      = 0.0f;   /* not level until the estimate says so */

    Altitude_Data_t altNow    = {0};
    Thrust_Data_t   thrustNow = {0};
    Position_Data_t posNow    = {0};

    while(1){
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(RC_TICK_MS));

        /* Recovers its radio by itself, both at power-up and after a supply dip in flight */
        TickType_t tickNow = xTaskGetTickCount();
        if((TickType_t)(tickNow - lastFrame) >= pdMS_TO_TICKS(RC_RADIO_CHECK_MS) &&
           (TickType_t)(tickNow - lastRadioCheck) >= pdMS_TO_TICKS(RC_RADIO_CHECK_MS)){
            lastRadioCheck = tickNow;
            if(!radioOk || (NRF24_Check() != NRF24_OK && NRF24_Check() != NRF24_OK)){
                radioOk = (NRF24_Init() == NRF24_OK);
            }
        }

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
        }

        (void)AltitudeTopic_Copy(&altNow, 0);
        (void)PositionTopic_Copy(&posNow);
        FlightMode_t activeNow = FlightMode_GetActive();
        TickType_t now = xTaskGetTickCount();
        uint8_t altValid = RC_Usable(&altTimer, altNow.valid,
                                     activeNow == FLIGHT_MODE_ALT_HOLD || 
                                     activeNow == FLIGHT_MODE_POS_HOLD, now);
        if(!Arm_IsAirborne())                        posDropped = 0;
        else if(posTimer.wasValid && !posNow.valid)  posDropped = 1;
        uint8_t posValid = RC_Usable(&posTimer, posNow.valid,
                                     activeNow == FLIGHT_MODE_POS_HOLD || !posDropped, now);

        /* Flight mode actually flown. On link loss: descend, holding the spot
         * (POS_HOLD) if the flow or the GPS gives a position, else ALT_HOLD if
         * the altitude estimate is good, otherwise ANGLE with a blind thrust cut. */
        FlightMode_t flightMode;
        if(linkOk) flightMode = FlightMode_Resolve(latest.mode, altValid, posValid);
        else       flightMode = FlightMode_Resolve(RC_MODE_POS_HOLD, altValid, posValid);
        rc.mode = (uint8_t)flightMode;
        FlightMode_SetActive(flightMode);

        uint8_t holdMode = (flightMode == FLIGHT_MODE_ALT_HOLD || flightMode == FLIGHT_MODE_POS_HOLD);
        uint8_t holdEngaged;
        float climbStickThrottle = RC_HoldThrottle(holdMode, linkOk, &holdEngaged);
        FlightMode_t stickMode = holdEngaged ? flightMode : FLIGHT_MODE_ANGLE;

        (void)ThrustTopic_Copy(&thrustNow, 0);

        Attitude_Data_t attNow;
        if(AttitudeTopic_Copy(&attNow) == pdPASS &&
           (attNow.q0 * attNow.q0 + attNow.q1 * attNow.q1 + attNow.q2 * attNow.q2 + attNow.q3 * attNow.q3) > 0.5f){
            cosTilt = 1.0f - 2.0f * (attNow.q1 * attNow.q1 + attNow.q2 * attNow.q2);
        }

        Arm_Input_t armIn;
        armIn.armSwitch      = (latest.flags & RC_FLAG_ARM) ? 1U : 0U;
        armIn.sensorsReady   = Calib_ReadyToArm();;
        armIn.linkOk         = linkOk;
        armIn.linkLostMs     = (uint32_t)(age * portTICK_PERIOD_MS);
        armIn.throttleLow    = (rc.throttle < RC_ARM_THROTTLE_MAX);
        armIn.takeoffRequest = linkOk && FlightMode_TakeoffRequested(stickMode, climbStickThrottle);
        armIn.descendRequest = linkOk && FlightMode_DescendRequested(stickMode, climbStickThrottle);
        armIn.thrust         = thrustNow.thrust;
        armIn.hoverThrust    = thrustNow.hoverThrust;
        armIn.liftVz         = altNow.liftVz;
        armIn.accelUp        = altNow.accelUp;
        armIn.altitudeValid  = altNow.valid;
        armIn.onFloor        = altNow.onFloor;
        armIn.cosTilt        = cosTilt;

        Arm_Update(&armIn);
        climbStickThrottle = RC_HoldThrottle(holdMode, linkOk, &holdEngaged);   /* AIRBORNE may have come just now */

        AttitudeSetpoint_Data_t attSp = {0};
        AltitudeSetpoint_Data_t altSp = {0};
        altSp.mode     = (uint8_t)flightMode;
        altSp.holdLost = linkOk && latest.mode != RC_MODE_ANGLE && flightMode == FLIGHT_MODE_ANGLE;

        if(linkOk){
            attSp.roll    = rc.roll  * RC_MAX_TILT_RAD;
            attSp.pitch   = rc.pitch * RC_MAX_TILT_RAD;
            attSp.yawRate = rc.yaw   * RC_MAX_YAW_RATE;

            switch(flightMode){
            case FLIGHT_MODE_POS_HOLD:
                /* roll/pitch: the position controller flies them (the sticks give
                 * it a speed); the stick angles above only stand in when its
                 * output stops. Vertical axis is ALT_HOLD. */
                /* fall through */
            case FLIGHT_MODE_ALT_HOLD:
                altSp.climbRate    = holdEngaged ? FlightMode_ClimbRate(climbStickThrottle) : 0.0f;
                altSp.manualThrust = rc.throttle;     /* the thrust until then, and if the estimate drops out */
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
