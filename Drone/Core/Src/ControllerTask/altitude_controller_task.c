#include "altitude_controller_task.h"
#include "altitude_topic.h"
#include "altitude_setpoint_topic.h"
#include "attitude_topic.h"
#include "thrust_topic.h"
#include "pid.h"
#include <math.h>

#define ALT_CTRL_RATE_HZ        100
#define ALT_CTRL_PERIOD_MS      (1000 / ALT_CTRL_RATE_HZ)
#define ALT_CTRL_DT             (1.0f / (float)ALT_CTRL_RATE_HZ)

#define ALT_KP                  1.0f    /* Not yet done, neet to do inspection*/
#define ALT_VZ_LIMIT            2.0f    /* m/s */
#define ALT_TARGET_MAX_ERROR    3.0f    /* m, stops the target running away */

#define VZ_KP                   0.15f   /* Not yet done, neet to do inspection*/
#define VZ_KI                   0.10f   /* Not yet done, neet to do inspection*/
#define VZ_KD                   0.0f    /* Not yet done, neet to do inspection*/
#define VZ_INTEGRAL_LIMIT       2.0f
#define VZ_OUTPUT_LIMIT         0.50f
#define VZ_D_CUTOFF_HZ          10.0f

#define HOVER_THRUST            0.35f   /* Not yet done, neet to do inspection*/

#define THRUST_MIN              0.10f
#define THRUST_MAX              0.85f
#define TILT_COMP_MIN           0.5f

#define BARO_LOSS_LIMIT_CYCLES  50      /* 0.5 s at 100 Hz */
#define ALT_CTRL_COPY_TIMEOUT   pdMS_TO_TICKS(2)

AltitudeController_Handle_t altitudeController;

static PID_t vzPID;

void AltitudeController_SetTaskHandle(TaskHandle_t handle){
	altitudeController.controllerTask = handle;
}

void AltitudeControllerTask(void *argument){
	AltitudeController_SetTaskHandle(xTaskGetCurrentTaskHandle());

    PID_Init(&vzPID, VZ_KP, VZ_KI, VZ_KD, VZ_INTEGRAL_LIMIT, VZ_OUTPUT_LIMIT, VZ_D_CUTOFF_HZ);
	Altitude_Data_t         altitude = {0};
	AltitudeSetpoint_Data_t setpoint = {0};
	Attitude_Data_t         attitude = {0};

    uint8_t  holdActive   = 0;
    uint16_t baroLostCycles = 0;

    TickType_t last = xTaskGetTickCount();

    while(1){
        vTaskDelayUntil(&last, pdMS_TO_TICKS(ALT_CTRL_PERIOD_MS));

        if(AltitudeSetpointTopic_Copy(&setpoint, ALT_CTRL_COPY_TIMEOUT) != pdPASS) continue;
        if(AltitudeTopic_Copy(&altitude, ALT_CTRL_COPY_TIMEOUT)         != pdPASS) continue;
        if(AttitudeTopic_Copy(&attitude)         						!= pdPASS) continue;

        if(altitude.baroValid) baroLostCycles = 0;
        else if(baroLostCycles < BARO_LOSS_LIMIT_CYCLES) baroLostCycles++;

        uint8_t estimateUsable = altitude.valid && (baroLostCycles < BARO_LOSS_LIMIT_CYCLES);
        uint8_t wantHold = setpoint.holdEnabled && estimateUsable;
        float thrust;

        if(!wantHold){
            thrust = setpoint.manualThrust;

            PID_Init(&vzPID, VZ_KP, VZ_KI, VZ_KD, VZ_INTEGRAL_LIMIT, VZ_OUTPUT_LIMIT, VZ_D_CUTOFF_HZ);
            holdActive = 0;
            altitudeController.holdTarget = altitude.altitude;
            altitudeController.vzSetpoint = 0.0f;
        }
        else{
            if(!holdActive){
                altitudeController.holdTarget = altitude.altitude;

                float tiltComp = cosf(attitude.roll) * cosf(attitude.pitch);
                if(tiltComp < TILT_COMP_MIN) tiltComp = TILT_COMP_MIN;

                if(VZ_KI > 0.0f){
                    vzPID.integral = (setpoint.manualThrust * tiltComp - HOVER_THRUST) / VZ_KI;
                    if(vzPID.integral >  VZ_INTEGRAL_LIMIT) vzPID.integral =  VZ_INTEGRAL_LIMIT;
                    if(vzPID.integral < -VZ_INTEGRAL_LIMIT) vzPID.integral = -VZ_INTEGRAL_LIMIT;
                }
                vzPID.prevMeasurement = altitude.verticalSpeed;
                vzPID.dFiltered       = 0.0f;
                vzPID.seeded          = 1;

                holdActive = 1;
            }

            altitudeController.holdTarget += setpoint.climbRate * ALT_CTRL_DT;

            float targetError = altitudeController.holdTarget - altitude.altitude;
            if(targetError >  ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude + ALT_TARGET_MAX_ERROR;
            if(targetError < -ALT_TARGET_MAX_ERROR)
                altitudeController.holdTarget = altitude.altitude - ALT_TARGET_MAX_ERROR;

            float vzSetpoint = ALT_KP * (altitudeController.holdTarget - altitude.altitude) + setpoint.climbRate;

            if(vzSetpoint >  ALT_VZ_LIMIT) vzSetpoint =  ALT_VZ_LIMIT;
            if(vzSetpoint < -ALT_VZ_LIMIT) vzSetpoint = -ALT_VZ_LIMIT;
            altitudeController.vzSetpoint = vzSetpoint;

            thrust = HOVER_THRUST + PID_Update(&vzPID, vzSetpoint, altitude.verticalSpeed, ALT_CTRL_DT);

            /* Increase thrust to compensate for the drone tilting */
            float tiltComp = cosf(attitude.roll) * cosf(attitude.pitch);
            if(tiltComp < TILT_COMP_MIN) tiltComp = TILT_COMP_MIN;
            thrust /= tiltComp;
        }
        if(thrust > THRUST_MAX) thrust = THRUST_MAX;
        if(thrust < THRUST_MIN) thrust = THRUST_MIN;

        Thrust_Data_t *out = &altitudeController.data;
        out->thrust       = thrust;
        out->holdActive   = holdActive;
        out->timestamp_us = altitude.timestamp_us;

        ThrustTopic_Publish(out);
    }
}
