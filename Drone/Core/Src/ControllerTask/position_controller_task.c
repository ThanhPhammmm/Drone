#include "position_controller_task.h"

#define POS_CTRL_DT_MAX             0.05f   /* s, cap after a long stall */

/* Sticks give a speed in the drone's own frame: forward / right, as in ANGLE */
#define POS_CTRL_VEL_MAX            1.0f    /* m/s at full stick */
#define POS_CTRL_STICK_DEADBAND     0.05f   /* the remote has its own; this only stops stick noise */

/* Flow (indoors, low): a clean, quick velocity */
#define POS_CTRL_FLOW_POS_KP             1.0f    /* 1/s: position error -> speed */
#define POS_CTRL_FLOW_VEL_KP             1.5f    /* m/s^2 per m/s; 2.0 swings when the flow scale is 40 % off (sim) */
#define POS_CTRL_FLOW_VEL_KI             0.5f    /* 1/s: holds against a steady push (draught, weight off centre) */
#define POS_CTRL_FLOW_I_MAX              1.0f    /* m/s^2 the I-term may hold (~6 deg of tilt) */

/* GPS (outdoors, high): the position wanders ~1 m and comes 0.15 s late, so
 * softer; wind needs more I */
#define POS_CTRL_GPS_POS_KP         0.5f
#define POS_CTRL_GPS_VEL_KP         1.0f
#define POS_CTRL_GPS_VEL_KI         0.3f
#define POS_CTRL_GPS_I_MAX          1.5f    /* ~9 deg of tilt */

#define POS_CTRL_ACCEL_MAX          2.5f    /* m/s^2, ~14 deg of tilt */
#define POS_CTRL_TILT_MAX           0.30f   /* rad */

#define POS_CTRL_STOPPED_VEL        0.15f   /* m/s: slow enough to hold where it is... */
#define POS_CTRL_BRAKE_MIN_S        0.5f    /* ...after braking this long: a quick flick ends before the drone gets going */
#define POS_CTRL_BRAKE_MAX_S        2.0f    /* hold where it is after this long anyway */

#define GRAVITY                     9.80665f

static const PosGains_t gainsFlow = { POS_CTRL_FLOW_POS_KP, POS_CTRL_FLOW_VEL_KP, POS_CTRL_FLOW_VEL_KI, POS_CTRL_FLOW_I_MAX };
static const PosGains_t gainsGps  = { POS_CTRL_GPS_POS_KP, POS_CTRL_GPS_VEL_KP, POS_CTRL_GPS_VEL_KI, POS_CTRL_GPS_I_MAX };

PositionController_Handle_t positionController;
static PosCtrl_t ctrl;

void PositionController_SetTaskHandle(TaskHandle_t handle){
	positionController.controllerTask = handle;
}

/* stick -1..1 -> -1..1 with a dead band in the middle */
static float PositionController_Stick(float s){
	if(s > -POS_CTRL_STICK_DEADBAND && s < POS_CTRL_STICK_DEADBAND) return 0.0f;
	s = (s > 0.0f) ? (s - POS_CTRL_STICK_DEADBAND) : (s + POS_CTRL_STICK_DEADBAND);
	s /= (1.0f - POS_CTRL_STICK_DEADBAND);
	if(s >  1.0f) s =  1.0f;
	if(s < -1.0f) s = -1.0f;
	return s;
}

/* scale (x, y) down to length max at most */
static void PositionController_Limit(float *x, float *y, float max){
	float len = sqrtf((*x) * (*x) + (*y) * (*y));
	if(len > max){
		*x *= max / len;
		*y *= max / len;
	}
}

static float PositionController_Clamp(float v, float max){
	return (v > max) ? max : ((v < -max) ? -max : v);
}

static void PositionController_Step(void){
	Position_Data_t pos;
	if(PositionTopic_Copy(&pos) != pdPASS) return;
	if(ctrl.lastUs != 0 && pos.timestamp_us == ctrl.lastUs) return;

	float dt = (ctrl.lastUs != 0) ? (float)(uint32_t)(pos.timestamp_us - ctrl.lastUs) * 1e-6f : 0.0f;
	ctrl.lastUs = pos.timestamp_us;
	if(dt > POS_CTRL_DT_MAX) dt = POS_CTRL_DT_MAX;

	RC_Data_t rcNow;
	if(RCTopic_Copy(&rcNow, 0) == pdPASS) ctrl.rc = rcNow;

	Tilt_Data_t *out = &positionController.data;
	out->timestamp_us = pos.timestamp_us;

	uint8_t active = (FlightMode_GetActive() == FLIGHT_MODE_POS_HOLD) && Arm_IsAirborne() && pos.valid;
	if(!active){
		ctrl.engaged = 0;
		out->roll    = 0.0f;
		out->pitch   = 0.0f;
		out->active  = 0;
		positionController.targetX    = pos.x;     /* no target, it is where it is */
		positionController.targetY    = pos.y;
		positionController.vxSetpoint = 0.0f;
		positionController.vySetpoint = 0.0f;
		TiltTopic_Publish(out);
		return;
	}

	if(!ctrl.engaged){
		/* it may still be drifting when the hold starts: stop first, then hold */
		ctrl.engaged    = 1;
		ctrl.phase      = POS_PHASE_BRAKE;
		ctrl.brakeTime  = 0.0f;
		ctrl.iAccelX    = 0.0f;
		ctrl.iAccelY    = 0.0f;
		positionController.targetX = pos.x;
		positionController.targetY = pos.y;
	}

	const PosGains_t *k = (pos.source == POS_SOURCE_GPS) ? &gainsGps : &gainsFlow;

	float sFwd   = PositionController_Stick(-ctrl.rc.pitch);
	float sRight = PositionController_Stick( ctrl.rc.roll);

	float c = cosf(pos.yaw);
	float s = sinf(pos.yaw);

	float vxSp = 0.0f;
	float vySp = 0.0f;

	if(sFwd != 0.0f || sRight != 0.0f){
		ctrl.phase = POS_PHASE_MOVE;
		vxSp = (c * sFwd - s * sRight) * POS_CTRL_VEL_MAX;
		vySp = (s * sFwd + c * sRight) * POS_CTRL_VEL_MAX;
		PositionController_Limit(&vxSp, &vySp, POS_CTRL_VEL_MAX);
	}
	else{
		if(ctrl.phase == POS_PHASE_MOVE){
			ctrl.phase     = POS_PHASE_BRAKE;
			ctrl.brakeTime = 0.0f;
		}
		if(ctrl.phase == POS_PHASE_BRAKE){
			/* speed setpoint 0 until it has nearly stopped: holding the spot where
			 * the stick came back would pull it back there after the stop */
			ctrl.brakeTime += dt;
			float speed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
			if((speed < POS_CTRL_STOPPED_VEL && ctrl.brakeTime > POS_CTRL_BRAKE_MIN_S) ||
			   ctrl.brakeTime > POS_CTRL_BRAKE_MAX_S){
				ctrl.phase = POS_PHASE_HOLD;
				positionController.targetX = pos.x;
				positionController.targetY = pos.y;
			}
		}
		if(ctrl.phase == POS_PHASE_HOLD){
			vxSp = k->posKp * (positionController.targetX - pos.x);
			vySp = k->posKp * (positionController.targetY - pos.y);
			PositionController_Limit(&vxSp, &vySp, POS_CTRL_VEL_MAX);
		}
	}
	positionController.vxSetpoint = vxSp;
	positionController.vySetpoint = vySp;

	/* speed PI -> acceleration, level frame */
	float ex = vxSp - pos.vx;
	float ey = vySp - pos.vy;
	ctrl.iAccelX = PositionController_Clamp(ctrl.iAccelX + k->velKi * ex * dt, k->iMax);
	ctrl.iAccelY = PositionController_Clamp(ctrl.iAccelY + k->velKi * ey * dt, k->iMax);

	float ax = k->velKp * ex + ctrl.iAccelX;
	float ay = k->velKp * ey + ctrl.iAccelY;
	PositionController_Limit(&ax, &ay, POS_CTRL_ACCEL_MAX);

	/* level frame -> heading frame, then the tilt that gives that acceleration */
	float aFwd   =  c * ax + s * ay;
	float aRight = -s * ax + c * ay;

	float pitch = -atanf(aFwd / GRAVITY);
	float roll  =  atanf(aRight * cosf(pitch) / GRAVITY);

	out->roll   = PositionController_Clamp(roll,  POS_CTRL_TILT_MAX);
	out->pitch  = PositionController_Clamp(pitch, POS_CTRL_TILT_MAX);
	out->active = 1;
	TiltTopic_Publish(out);
}

void PositionControllerTask(void *argument){
	PositionController_SetTaskHandle(xTaskGetCurrentTaskHandle());
	PositionTopic_Subscribe(positionController.controllerTask, POSITION_CONTROLLER_ID_TASK);

	/* If the estimates stop, so does the tilt output, 
	* and the attitude controller goes back to the sticks. */
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		PositionController_Step();
	}
}
