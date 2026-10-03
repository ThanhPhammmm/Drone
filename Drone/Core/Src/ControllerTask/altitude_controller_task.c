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
#define VZ_OUTPUT_LIMIT         0.50f	/* VZ_OUTPUT_LIMIT = THRUST_MAX - HOVER_THRUST */
#define VZ_D_CUTOFF_HZ          10.0f

#define HOVER_THRUST            0.35f   /* Not yet done, need to do inspection*/

#define THRUST_MIN              0.10f
#define THRUST_MAX              0.85f
#define TILT_COMP_MIN           0.5f

#define TAKEOFF_RAMP_RATE       0.25f   /* thrust per second */
#define TAKEOFF_RAMP_MAX        (HOVER_THRUST + 0.10f)
#define TAKEOFF_LIFTOFF_VZ      0.15f   /* m/s */
#define TAKEOFF_LIFTOFF_ALT     0.10f   /* m above where the ramp started */
#define TAKEOFF_CLIMB_RATE      0.5f    /* m/s, minimum climb until the arm state says AIRBORNE */

#define ALT_CTRL_COPY_TIMEOUT   pdMS_TO_TICKS(2)

typedef enum{
	ALT_PHASE_OFF = 0,		/* controllers do not own the motors: thrust 0 */
	ALT_PHASE_MANUAL,		/* ANGLE: stick is collective thrust */
	ALT_PHASE_RAMP,			/* ALT_HOLD take-off spool-up */
	ALT_PHASE_HOLD			/* ALT_HOLD closed loop */
} AltPhase_t;

AltitudeController_Handle_t altitudeController;

static PID_t vzPID;

void AltitudeController_SetTaskHandle(TaskHandle_t handle){
	altitudeController.controllerTask = handle;
}

/* Bumpless entry into the closed loop: preload the integrator so the first
 * output equals the thrust that was being flown. */
static void AltitudeController_EngageHold(const Altitude_Data_t *altitude, float thrustNow, float tiltComp){
	PID_Reset(&vzPID);

	altitudeController.holdTarget = altitude->altitude;

	if(VZ_KI > 0.0f){
		vzPID.integral = (thrustNow * tiltComp - HOVER_THRUST) / VZ_KI;
		if(vzPID.integral >  VZ_INTEGRAL_LIMIT) vzPID.integral =  VZ_INTEGRAL_LIMIT;
		if(vzPID.integral < -VZ_INTEGRAL_LIMIT) vzPID.integral = -VZ_INTEGRAL_LIMIT;
		vzPID.prevIntegral = vzPID.integral;
	}
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
    float      rampThrust   = 0.0f;
    float      rampStartAlt = 0.0f;

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

        FlightMode_t mode  = (FlightMode_t)setpoint.mode;
        arm_state_t  state = Arm_GetState();

        /* POS_HOLD flies ALT_HOLD on the vertical axis. Without a usable
         * estimate the stick is flown as thrust (ANGLE). */
        uint8_t altHold = (mode == FLIGHT_MODE_ALT_HOLD || mode == FLIGHT_MODE_POS_HOLD) && altitude.valid;

        float tiltComp = cosf(attitude.roll) * cosf(attitude.pitch);
        if(tiltComp < TILT_COMP_MIN) tiltComp = TILT_COMP_MIN;

        float thrust;
        float vzError = 0.0f;
        uint8_t thrustSaturatedHigh = 0;
        uint8_t thrustSaturatedLow  = 0;

        if(!Arm_MotorsAllowed()){
            phase  = ALT_PHASE_OFF;
            thrust = 0.0f;
        }
        else if(!altHold){
            phase  = ALT_PHASE_MANUAL;
            thrust = setpoint.manualThrust;
        }
        else if(state == FLIGHT_TAKEOFF && phase != ALT_PHASE_HOLD){
            if(phase != ALT_PHASE_RAMP){
                phase        = ALT_PHASE_RAMP;
                rampThrust   = (lastThrust > THRUST_MIN) ? lastThrust : THRUST_MIN;
                rampStartAlt = altitude.altitude;
            }

            rampThrust += TAKEOFF_RAMP_RATE * dt;
            if(rampThrust > TAKEOFF_RAMP_MAX) rampThrust = TAKEOFF_RAMP_MAX;
            thrust = rampThrust;

            uint8_t liftoff = (altitude.verticalSpeed > TAKEOFF_LIFTOFF_VZ) ||
                              ((altitude.altitude - rampStartAlt) > TAKEOFF_LIFTOFF_ALT);
            if(liftoff){
                AltitudeController_EngageHold(&altitude, rampThrust, tiltComp);
                phase = ALT_PHASE_HOLD;
            }
        }
        else{
            if(phase != ALT_PHASE_HOLD){
                AltitudeController_EngageHold(&altitude, lastThrust, tiltComp);
                phase = ALT_PHASE_HOLD;
            }

            float climbRate = setpoint.climbRate;
            if(state == FLIGHT_TAKEOFF){
                /* just off the ground: keep climbing until AIRBORNE is confirmed,
                 * and track the altitude instead of holding it */
                if(climbRate < TAKEOFF_CLIMB_RATE) climbRate = TAKEOFF_CLIMB_RATE;
                altitudeController.holdTarget = altitude.altitude;
            }

            altitudeController.holdTarget += climbRate * dt;

            float targetError = altitudeController.holdTarget - altitude.altitude;
            if(targetError >  ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude + ALT_TARGET_MAX_ERROR;
            if(targetError < -ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude - ALT_TARGET_MAX_ERROR;

            float vzSetpoint = ALT_KP * (altitudeController.holdTarget - altitude.altitude) + climbRate;

            if(vzSetpoint >  ALT_VZ_LIMIT) vzSetpoint =  ALT_VZ_LIMIT;
            if(vzSetpoint < -ALT_VZ_LIMIT) vzSetpoint = -ALT_VZ_LIMIT;
            altitudeController.vzSetpoint = vzSetpoint;

            thrust = HOVER_THRUST + PID_Update(&vzPID, vzSetpoint, altitude.verticalSpeed, dt);
            vzError = vzSetpoint - altitude.verticalSpeed;

            /* Increase thrust to compensate for the drone tilting */
            thrust /= tiltComp;
        }

        if(phase != ALT_PHASE_HOLD){
            PID_Reset(&vzPID);
            altitudeController.holdTarget = altitude.altitude;
            altitudeController.vzSetpoint = 0.0f;
        }

        if(thrust > THRUST_MAX){
            thrust = THRUST_MAX;
            thrustSaturatedHigh = 1;
        }
        else if(thrust > 0.0f && thrust < THRUST_MIN){
            thrust = THRUST_MIN;
            thrustSaturatedLow = 1;
        }
        else if(thrust < 0.0f) thrust = 0.0f;

        if(phase == ALT_PHASE_HOLD){
            PID_NotifySaturation(&vzPID, vzError, thrustSaturatedHigh, thrustSaturatedLow);
        }

        lastThrust = thrust;

        Thrust_Data_t *out = &altitudeController.data;
        out->thrust       = thrust;
        out->holdActive   = (phase == ALT_PHASE_HOLD);
        out->timestamp_us = altitude.timestamp_us;

        ThrustTopic_Publish(out);
    }
}
