#include "altitude_controller_task.h"

#define ALT_CTRL_RATE_HZ        100
#define ALT_CTRL_PERIOD_MS      (1000 / ALT_CTRL_RATE_HZ)
#define dt_MAX                  0.05f   /* s, cap after a long stall */

#define ALT_KP                  1.0f    /* Not yet done, need to do inspection*/
#define ALT_VZ_LIMIT            2.0f    /* m/s */
#define ALT_TARGET_MAX_ERROR    3.0f    /* m, stops the target running away */

#define VZ_KP                   0.15f   /* Not yet done, need to do inspection*/
#define VZ_KI                   0.10f   /* Not yet done, need to do inspection*/
#define VZ_KD                   0.0f    /* Not yet done, need to do inspection*/
#define VZ_INTEGRAL_LIMIT       2.0f
#define VZ_OUTPUT_LIMIT         0.50f	/* most the loop may move the thrust away from the hover thrust */
#define VZ_D_CUTOFF_HZ          10.0f

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

#define ALT_CTRL_COPY_TIMEOUT   pdMS_TO_TICKS(2)

typedef enum{
	ALT_PHASE_OFF = 0,		/* controllers do not own the motors: thrust 0 */
	ALT_PHASE_MANUAL,		/* ANGLE, and every take-off: stick is collective thrust */
	ALT_PHASE_HOLD			/* ALT_HOLD closed loop, once airborne */
} AltPhase_t;

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

	float sample = thrustNow * tiltComp * powf(GRAVITY / (GRAVITY + a), HOVER_EST_EXP);
	if(!hoverEstValid){
		hoverEst      = sample;
		hoverEstValid = 1;
	}
	else{
		hoverEst += (sample - hoverEst) * dt / (HOVER_EST_TAU_S + dt);
	}
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

    PID_Init(&vzPID, VZ_KP, VZ_KI, VZ_KD, VZ_INTEGRAL_LIMIT, VZ_OUTPUT_LIMIT, VZ_D_CUTOFF_HZ);
	Altitude_Data_t         altitude = {0};
	AltitudeSetpoint_Data_t setpoint = {0};
	Attitude_Data_t         attitude = {0};

    AltPhase_t phase        = ALT_PHASE_OFF;
    float      lastThrust   = 0.0f;
    uint8_t    handover     = 0;    /* MANUAL thrust still sliding over from the hold thrust */
    uint8_t    spoolUp      = 0;    /* take-off: thrust rise still rate-limited */
    uint8_t    takeoffClimb = 0;    /* ALT_HOLD take-off: still climbing to TAKEOFF_CLIMB_M */
    uint8_t    wasTakingOff = 0;    /* arm state was FLIGHT_TAKEOFF on the previous run */
    float      hoverRef     = 0.0f; /* published hover thrust (landing detector), 0 = not known */

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

        float tiltComp = cosf(attitude.roll) * cosf(attitude.pitch);
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
        }
        else if(!altHold){
            if(phase == ALT_PHASE_HOLD) handover = 1; /* Hold -> Angle */
            phase        = ALT_PHASE_MANUAL;
            thrust       = setpoint.manualThrust;
            takeoffClimb = 0;

            if(handover){
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
                AltitudeController_EngageHold(&altitude, lastThrust, tiltComp);
                phase        = ALT_PHASE_HOLD;
                spoolUp      = 0;
                takeoffClimb = wasTakingOff;    /* straight from the take-off */
                if(takeoffClimb) altitudeController.holdTarget += TAKEOFF_CLIMB_M;
            }

            float climbRate = setpoint.climbRate;
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
            if(vzSetpoint >  vzLimit)      vzSetpoint =  vzLimit;
            if(vzSetpoint < -ALT_VZ_LIMIT) vzSetpoint = -ALT_VZ_LIMIT;
            altitudeController.vzSetpoint = vzSetpoint;

            thrust = altitudeController.hoverThrust + PID_Update(&vzPID, vzSetpoint, altitude.verticalSpeed, dt);
            vzError = vzSetpoint - altitude.verticalSpeed;

            /* Increase thrust to compensate for the drone tilting */
            thrust /= tiltComp;
        }

        if(phase != ALT_PHASE_HOLD){
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
            if(phase == ALT_PHASE_HOLD) hoverRef = altitudeController.hoverThrust + vzPID.ki * vzPID.integral;
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
