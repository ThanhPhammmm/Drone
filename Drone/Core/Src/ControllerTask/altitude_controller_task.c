#include "altitude_controller_task.h"

#define ALT_CTRL_RATE_HZ        100
#define ALT_CTRL_PERIOD_MS      (1000 / ALT_CTRL_RATE_HZ)
#define dt_MAX                  0.05f   /* s, cap after a long stall */

#define ALT_KP                  1.0f    /* 1/s: height error -> climb speed */
#define ALT_VZ_LIMIT            2.0f    /* m/s, fastest climb */
#define ALT_TARGET_MAX_ERROR    3.0f    /* m, stops the target running away */

/* Climb-speed loop. Its output is a part of the hover thrust (0.1 = 10 % more
 * than hover), so the same gains suit a light and a heavy drone. */
#define VZ_KP                   0.30f   /* per m/s */
#define VZ_KI                   0.20f   /* per m */
#define VZ_KD                   0.0f
#define VZ_I_MAX                0.40f   /* the I-term moves the hover thrust by 40 % at most */
#define VZ_OUTPUT_LIMIT         0.60f
#define VZ_D_CUTOFF_HZ          10.0f

#define DESCENT_VZ_PER_M        0.5f    /* 1/s: allowed descent speed per metre of height */
#define DESCENT_VZ_MAX          1.0f    /* m/s, high up */
#define DESCENT_VZ_MIN          0.25f   /* m/s, the touchdown speed and the slowest descent: 0.1 was
                                         * smaller than the climb-speed error near the floor, so the
                                         * drone hung 20 cm up with the stick fully down */

#define LAND_CUT_RATE           1.0f    /* standing on it: the thrust goes down by this part per s */

#define THRUST_MIN              0.10f
#define THRUST_MAX              0.85f
#define TILT_COMP_MIN           0.5f

#define HOVER_EST_TAU_S         0.4f    /* s */
#define HOVER_EST_ACCEL_MAX     4.0f    /* m/s^2, samples beyond (bumps, landing) are skipped */
#define HOVER_EST_EXP           0.6f    /* thrust ~ command^1.6..1.8 for typical motors and props */

#define HOVER_EST_LIFT_VZ       0.10f   /* m/s, accelerometer lift-off speed: off the ground */
#define GRAVITY                 9.80665f

#define TAKEOFF_SPOOL_RATE      0.30f
#define TAKEOFF_SPOOL_FROM      0.05f   /* = MOTOR_IDLE in Actuator.c */

#define TAKEOFF_CLIMB_M         1.0f
#define TAKEOFF_CLIMB_VZ        0.5f    /* m/s */
#define TAKEOFF_CLIMB_DONE_M    0.10f

#define MANUAL_HANDOVER_RATE    0.5f
#define FALLBACK_STICK_MOVE     0.05f   /* hold lost by itself: the thrust stays until the throttle moves this much */

#define ALT_STALL_THRUST_BAND   0.06f   /* +-6 % of the thrust, ~+-1 m/s^2 */

#define ALT_CTRL_COPY_TIMEOUT   pdMS_TO_TICKS(2)

AltitudeController_Handle_t altitudeController;

static PID_t vzPID;

void AltitudeController_SetTaskHandle(TaskHandle_t handle){
	altitudeController.controllerTask = handle;
}

static float   hoverEst      = 0.0f;
static uint8_t hoverEstValid = 0;

static void AltitudeController_LearnHover(const Altitude_Data_t *altitude, float thrustNow, float tiltComp, float dt){
	float a = altitude->accelUp;
	if(thrustNow <= THRUST_MIN || a > HOVER_EST_ACCEL_MAX || a < -HOVER_EST_ACCEL_MAX) return;

    /*
        Fcurrent = m*(g+a)
        Fhover = m*g
        =>Fhover/Fcurrent = g/(g+a)
        =>Fhover = Fcurrent*(g/(g+a))
    */
	float sample = thrustNow * tiltComp * powf(GRAVITY / (GRAVITY + a), HOVER_EST_EXP);
	if(!hoverEstValid){
		hoverEst      = sample;
		hoverEstValid = 1;
	}
	else{
		hoverEst += (sample - hoverEst) * dt / (HOVER_EST_TAU_S + dt);
	}
}

static float AltitudeController_DescentLimit(const Altitude_Data_t *altitude){
	float v = DESCENT_VZ_PER_M * altitude->aboveFloor;
	if(v < DESCENT_VZ_MIN) v = DESCENT_VZ_MIN;
	if(v > DESCENT_VZ_MAX) v = DESCENT_VZ_MAX;
	return v;
}

static void AltitudeController_EngageHold(const Altitude_Data_t *altitude, float thrustNow, float tiltComp){
	PID_Reset(&vzPID);

	altitudeController.holdTarget = altitude->altitude;

	float hover = hoverEstValid ? hoverEst : thrustNow * tiltComp;
	if(hover < THRUST_MIN) hover = THRUST_MIN;
	if(hover > THRUST_MAX) hover = THRUST_MAX;
	altitudeController.hoverThrust = hover;

	vzPID.prevMeasurement = altitude->verticalSpeed;
	vzPID.dFiltered       = 0.0f;
	vzPID.seeded          = 1;
}

void AltitudeControllerTask(void *argument){
	AltitudeController_SetTaskHandle(xTaskGetCurrentTaskHandle());

    PID_Init(&vzPID, VZ_KP, VZ_KI, VZ_KD, VZ_I_MAX / VZ_KI, VZ_OUTPUT_LIMIT, VZ_D_CUTOFF_HZ);
	Altitude_Data_t         altitude = {0};
	AltitudeSetpoint_Data_t setpoint = {0};
	Attitude_Data_t         attitude = {0};

    AltPhase_t phase        = ALT_PHASE_OFF;
    float      lastThrust   = 0.0f;
    uint8_t    handover     = 0;    /* MANUAL thrust still sliding over from the hold thrust */
    uint8_t    spoolUp      = 0;    /* take-off: thrust rise still rate-limited */
    uint8_t    takeoffClimb = 0;    /* ALT_HOLD take-off: still climbing to TAKEOFF_CLIMB_M */
    uint8_t    wasTakingOff = 0;    /* arm state was FLIGHT_TAKEOFF on the previous run */
    uint8_t    keepThrust   = 0;    /* hold lost by itself: keep the hold thrust until the pilot moves the throttle */
    float      keepStick    = 0.0f; /* the throttle stick when it was lost */
    float      hoverRef     = 0.0f; /* published hover thrust (landing detector), 0 = not known */
    float      landCut      = 1.0f; /* landing and standing on the floor: thrust taken down to this part */
    uint8_t    stallHold    = 0;    /* range sensor stalled while hovering: thrust kept near stallThrust */
    float      stallThrust  = 0.0f; /* the thrust that held the height when it stalled */

    TickType_t last = xTaskGetTickCount();
    TickType_t lastRun = last;

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ALT_CTRL_PERIOD_MS));

        if(AltitudeSetpointTopic_Copy(&setpoint, ALT_CTRL_COPY_TIMEOUT) != pdPASS) continue;
        if(AltitudeTopic_Copy(&altitude, ALT_CTRL_COPY_TIMEOUT)         != pdPASS) continue;
        if(AttitudeTopic_Copy(&attitude)         						!= pdPASS) continue;

        float dt = (float)(last - lastRun) * ((float)portTICK_PERIOD_MS * 0.001f);
        lastRun = last;
        if(dt > dt_MAX) dt = dt_MAX;

        FlightMode_t mode      = (FlightMode_t)setpoint.mode;
        arm_state_t  armState  = Arm_GetState();
        uint8_t      takingOff = (armState == FLIGHT_TAKEOFF);

        uint8_t altHold = (mode == FLIGHT_MODE_ALT_HOLD || mode == FLIGHT_MODE_POS_HOLD) &&
                          altitude.valid && Arm_IsAirborne();

        float tiltComp = 1.0f - 2.0f * (attitude.q1 * attitude.q1 + attitude.q2 * attitude.q2);
        if(tiltComp < TILT_COMP_MIN) tiltComp = TILT_COMP_MIN;

        float thrust;
        float vzError = 0.0f;
        uint8_t thrustSaturatedHigh = 0;
        uint8_t thrustSaturatedLow  = 0;

        if(!Arm_MotorsAllowed()){
            phase         = ALT_PHASE_OFF;
            thrust        = 0.0f;
            handover      = 0;
            hoverEstValid = 0;
            spoolUp       = 1;
            takeoffClimb  = 0;
            hoverRef      = 0.0f;
            keepThrust    = 0;
        }
        else if(!altHold){
            if(phase == ALT_PHASE_HOLD){
                /* Hold -> Angle */
                handover = 1;

                /* The hover thrust the hold really used (its I-term has followed the
                 * battery): the next hold starts from it, not from an old estimate. */
                float used = altitudeController.hoverThrust * (1.0f + vzPID.ki * vzPID.integral);
                if(used < THRUST_MIN) used = THRUST_MIN;
                if(used > THRUST_MAX) used = THRUST_MAX;
                hoverEst      = used;
                hoverEstValid = 1;

                /* Not the pilot's choice (the altitude estimate dropped out): the stick
                 * is wherever it was for the hold, often centre = far from hover. */
                keepThrust = setpoint.holdLost;
                keepStick  = setpoint.manualThrust;
            }
            phase        = ALT_PHASE_MANUAL;
            thrust       = setpoint.manualThrust;
            takeoffClimb = 0;

            if(keepThrust){
                float moved = setpoint.manualThrust - keepStick;
                if(!setpoint.holdLost || moved > FALLBACK_STICK_MOVE || moved < -FALLBACK_STICK_MOVE){
                    keepThrust = 0;                 /* the pilot takes over: slide to the stick from here */
                }
                else{
                    thrust = lastThrust;
                }
            }

            if(keepThrust){
                /* nothing: the thrust stays */
            }
            else if(handover){
                float step = MANUAL_HANDOVER_RATE * dt;
                if(thrust > lastThrust + step)      thrust = lastThrust + step;
                else if(thrust < lastThrust - step) thrust = lastThrust - step;
                else                                handover = 0;
            }
            else if(spoolUp){
                float from = (lastThrust > TAKEOFF_SPOOL_FROM) ? lastThrust : TAKEOFF_SPOOL_FROM;
                float max  = from + TAKEOFF_SPOOL_RATE * dt;
                if(thrust > max) thrust = max;
                else if(Arm_IsAirborne()) spoolUp = 0;     /* caught up with the stick in the air */
            }
        }
        else{
            if(phase != ALT_PHASE_HOLD){
                keepThrust = 0;
                AltitudeController_EngageHold(&altitude, lastThrust, tiltComp);
                phase        = ALT_PHASE_HOLD;
                spoolUp      = 0;
                takeoffClimb = wasTakingOff;    /* straight from the take-off */
                landCut      = 1.0f;
                if(takeoffClimb) altitudeController.holdTarget += TAKEOFF_CLIMB_M;
            }

            /*
                * Descent speed: the lower, the slower.
                * The stick below centre sets the speed: just below the dead band = DESCENT_VZ_MIN,
                * fully down = the most the height allows. The failsafe always comes down at that most.
            */
            float descentMax = AltitudeController_DescentLimit(&altitude);

            float climbRate = setpoint.climbRate;
            if(climbRate < 0.0f){
                float part = (armState == FAILSAFE) ? 1.0f : (-climbRate / ALT_VZ_LIMIT);
                if(part > 1.0f) part = 1.0f;
                climbRate = -(DESCENT_VZ_MIN + part * (descentMax - DESCENT_VZ_MIN));
            }
            if(takeoffClimb){
                if(climbRate < 0.0f){
                    altitudeController.holdTarget = altitude.altitude;  /* the pilot takes over from here */
                    takeoffClimb = 0;
                }
                else if(altitudeController.holdTarget - altitude.altitude < TAKEOFF_CLIMB_DONE_M){
                    takeoffClimb = 0;
                }
                else{
                    climbRate = 0.0f;
                }
            }
            altitudeController.holdTarget += climbRate * dt;

            float targetError = altitudeController.holdTarget - altitude.altitude;
            if(targetError >  ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude + ALT_TARGET_MAX_ERROR;
            if(targetError < -ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude - ALT_TARGET_MAX_ERROR;

            float vzSetpoint = ALT_KP * (altitudeController.holdTarget - altitude.altitude) + climbRate;

            float vzLimit = takeoffClimb ? TAKEOFF_CLIMB_VZ : ALT_VZ_LIMIT;
            if(vzSetpoint >  vzLimit)       vzSetpoint =  vzLimit;
            if(vzSetpoint < -descentMax)    vzSetpoint = -descentMax;;
            altitudeController.vzSetpoint = vzSetpoint;

            /* landing and standing on the floor: the climb-speed error can never
             * go away, so take the thrust down for the land detector in arm.c */
            uint8_t touching = (armState == LANDING || armState == FAILSAFE) && altitude.onFloor && vzSetpoint < 0.0f;
            landCut += (touching ? -LAND_CUT_RATE : LAND_CUT_RATE) * dt;
            if(landCut < 0.0f) landCut = 0.0f;
            if(landCut > 1.0f) landCut = 1.0f;

            /* the thrust that holds the height now, before this step moves the I-term */
            float holding = altitudeController.hoverThrust * (1.0f + vzPID.ki * vzPID.integral);

            thrust = altitudeController.hoverThrust * landCut *
                     (1.0f + PID_Update(&vzPID, vzSetpoint, altitude.verticalSpeed, dt));
            vzError = vzSetpoint - altitude.verticalSpeed;

            uint8_t stalled = altitude.rangeStalled && armState == AIRBORNE && !takeoffClimb && climbRate == 0.0f;
            if(stalled){
                if(!stallHold) stallThrust = holding;
                PID_HoldIntegrator(&vzPID);     /* what it would learn now is the guess's error */
                float lo = stallThrust * (1.0f - ALT_STALL_THRUST_BAND);
                float hi = stallThrust * (1.0f + ALT_STALL_THRUST_BAND);
                if(thrust < lo) thrust = lo;
                if(thrust > hi) thrust = hi;
            }
            stallHold = stalled;

            /* Increase thrust to compensate for the drone tilting */
            thrust /= tiltComp;
        }

        if(phase != ALT_PHASE_HOLD){
            stallHold = 0;
            PID_Reset(&vzPID);
            altitudeController.holdTarget = altitude.altitude;
            altitudeController.vzSetpoint = 0.0f;
        }

        uint8_t thrustFloor = (phase != ALT_PHASE_MANUAL) || Arm_IsAirborne();

        if(thrust > THRUST_MAX){
            thrust = THRUST_MAX;
            thrustSaturatedHigh = 1;
        }
        else if(thrustFloor && thrust > 0.0f && thrust < THRUST_MIN){
            thrust = THRUST_MIN;
            thrustSaturatedLow = 1;
        }
        else if(thrust < 0.0f) thrust = 0.0f;

        if(phase == ALT_PHASE_HOLD){
            PID_NotifySaturation(&vzPID, vzError, thrustSaturatedHigh, thrustSaturatedLow);
        }

        if(phase == ALT_PHASE_MANUAL && !handover){
            if(armState == AIRBORNE || (takingOff && altitude.liftVz >= HOVER_EST_LIFT_VZ)){
                AltitudeController_LearnHover(&altitude, thrust, tiltComp, dt);
            }
            else if(takingOff){
                hoverEstValid = 0;      /* still standing: start again with the first sample off the ground */
            }
        }

        if(armState == AIRBORNE){
            if(phase == ALT_PHASE_HOLD) hoverRef = altitudeController.hoverThrust * (1.0f + vzPID.ki * vzPID.integral);
            else if(hoverEstValid)      hoverRef = hoverEst;
        }

        wasTakingOff = takingOff;
        lastThrust   = thrust;

        Thrust_Data_t *out = &altitudeController.data;
        out->thrust       = thrust;
        out->hoverThrust  = hoverRef;
        out->holdActive   = (phase == ALT_PHASE_HOLD);
        out->timestamp_us = altitude.timestamp_us;

        ThrustTopic_Publish(out);
    }
}
