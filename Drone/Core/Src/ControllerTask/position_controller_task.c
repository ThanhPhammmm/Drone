#include "position_controller_task.h"

#define POS_CTRL_DT_MAX             0.05f   /* s, cap after a long stall */

/* Sticks give a speed in the drone's own frame: forward / right, as in ANGLE */
#define POS_CTRL_VEL_MAX            1.0f    /* m/s at full stick */
#define POS_CTRL_STICK_DEADBAND     0.2f   /* the remote has its own; this only stops stick noise */

/* Flow (indoors, low): a clean, quick velocity */
#define POS_CTRL_FLOW_POS_KP		1.0f    /* 1/s: position error -> speed */
#define POS_CTRL_FLOW_VEL_KP		1.0f    /* m/s^2 per m/s; 2.0 swings when the flow scale is 40 % off (sim) */
#define POS_CTRL_FLOW_VEL_KI		0.5f    /* 1/s: holds against a steady push (draught, weight off centre) */
#define POS_CTRL_FLOW_I_MAX			1.0f    /* m/s^2 the I-term may hold (~6 deg of tilt) */

/* GPS (outdoors, high): the position wanders ~1 m and comes 0.15 s late, so
 * softer; wind needs more I */
#define POS_CTRL_GPS_POS_KP			0.5f
#define POS_CTRL_GPS_VEL_KP         1.0f
#define POS_CTRL_GPS_VEL_KI         0.3f
#define POS_CTRL_GPS_I_MAX          1.5f    /* ~9 deg of tilt */

#define POS_CTRL_ACCEL_MAX          2.5f    /* m/s^2, ~14 deg of tilt */
#define POS_CTRL_TILT_MAX           0.30f   /* rad */

#define POS_CTRL_TARGET_ACCEL       1.0f    /* m/s^2 the target gains and loses speed with (~6 deg of tilt) */
#define POS_CTRL_TARGET_LEAD_S      0.5f    /* s: a moving target is at most this long of its own speed ahead of
                                             * the drone (0.5 m at full stick, nothing once it has stopped), so
                                             * a drone that cannot follow is not pulled on after the stick is let go */
#define POS_CTRL_STOPPED_VEL        0.15f   /* m/s: the target has stopped and the drone is this slow: the target stays here... */
#define POS_CTRL_BRAKE_MAX_S        2.0f    /* ...or after this long anyway. Until then it waits on the drone */

/* Where the target starts when POS_HOLD comes on */
#define POS_CTRL_RESUME_S           3.0f    /* on within this long after lift-off, or back within this long
                                             * after it stopped (a short dropout)... */
#define POS_CTRL_RESUME_DIST        1.0f    /* ...the drone this close (m) to the spot it took off from / it held,
                                             * and the sticks not touched meanwhile: that spot is the target */

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

	float sFwd   = PositionController_Stick(-ctrl.rc.pitch);
	float sRight = PositionController_Stick( ctrl.rc.roll);

	Tilt_Data_t *out = &positionController.data;
	out->timestamp_us = pos.timestamp_us;

	uint8_t active = (FlightMode_GetActive() == FLIGHT_MODE_POS_HOLD) && Arm_IsAirborne() && pos.valid;
	if(!active){
		if(ctrl.engaged){
			/* just stopped (pilot switch, or the position dropped out): remember
			 * the spot, if the target was standing still on one */
			ctrl.resumable = (ctrl.vTargetX == 0.0f && ctrl.vTargetY == 0.0f && !ctrl.stopping);
			ctrl.savedX    = positionController.targetX;
			ctrl.savedY    = positionController.targetY;
			ctrl.stoppedUs = pos.timestamp_us;
		}
		if(!Arm_IsAirborne()){
			/* on the ground: the spot it stands on is the target after
			 * lift-off. */
			ctrl.resumable = 1;
			ctrl.savedX    = pos.x;
			ctrl.savedY    = pos.y;
			ctrl.stoppedUs = pos.timestamp_us;     /* so the time below counts from lift-off */
			ctrl.iAccelX   = 0.0f;
			ctrl.iAccelY   = 0.0f;
		}
		if(sFwd != 0.0f || sRight != 0.0f){
			ctrl.resumable = 0;     /* the pilot flew it somewhere else */
		}
		ctrl.engaged  = 0;
		ctrl.vTargetX = 0.0f;
		ctrl.vTargetY = 0.0f;
		ctrl.stopping = 0;
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
		ctrl.engaged = 1;

		float away   = sqrtf((pos.x - ctrl.savedX) * (pos.x - ctrl.savedX) + (pos.y - ctrl.savedY) * (pos.y - ctrl.savedY));
		float offFor = (float)(uint32_t)(pos.timestamp_us - ctrl.stoppedUs) * 1e-6f;
		if(ctrl.resumable && offFor < POS_CTRL_RESUME_S && away < POS_CTRL_RESUME_DIST){
			/* just off the ground: the target is the spot it took off from, so
			 * what it drifted since is flown back.
			 * Back after a short break: the spot it held, with its I-term (wind). */
			positionController.targetX = ctrl.savedX;
			positionController.targetY = ctrl.savedY;
			ctrl.vTargetX = 0.0f;
			ctrl.vTargetY = 0.0f;
			ctrl.stopping = 0;
		}
		else{
			/* anywhere else it may be moving: the target starts on the drone,
			 * at the drone's speed, and slows down from there. The drone stops
			 * with it instead of flying back to where the hold was switched on */
			positionController.targetX = pos.x;
			positionController.targetY = pos.y;
			ctrl.vTargetX = pos.vx;
			ctrl.vTargetY = pos.vy;
			PositionController_Limit(&ctrl.vTargetX, &ctrl.vTargetY, POS_CTRL_VEL_MAX);
			ctrl.stopping = 1;
			ctrl.stopTime = 0.0f;
			ctrl.iAccelX  = 0.0f;
			ctrl.iAccelY  = 0.0f;
		}
		ctrl.resumable = 0;
	}

	const PosGains_t *k = (pos.source == POS_SOURCE_GPS) ? &gainsGps : &gainsFlow;

	float c = cosf(pos.yaw);
	float s = sinf(pos.yaw);

	/* the speed the sticks ask for: heading frame -> level frame */
	float vStickX = (c * sFwd - s * sRight) * POS_CTRL_VEL_MAX;
	float vStickY = (s * sFwd + c * sRight) * POS_CTRL_VEL_MAX;
	PositionController_Limit(&vStickX, &vStickY, POS_CTRL_VEL_MAX);

	/* the target's speed goes there at POS_CTRL_TARGET_ACCEL, it does not jump.
	 * Stick let go: the target slows to a stop, and the drone stops with it */
	float dvx = vStickX - ctrl.vTargetX;
	float dvy = vStickY - ctrl.vTargetY;
	PositionController_Limit(&dvx, &dvy, POS_CTRL_TARGET_ACCEL * dt);
	ctrl.vTargetX += dvx;
	ctrl.vTargetY += dvy;

	/* the acceleration the target does: given to the drone directly, so it
	 * tilts with the stick and not only once it has fallen behind */
	float axTarget = (dt > 0.0f) ? dvx / dt : 0.0f;
	float ayTarget = (dt > 0.0f) ? dvy / dt : 0.0f;

	float targetSpeed = sqrtf(ctrl.vTargetX * ctrl.vTargetX + ctrl.vTargetY * ctrl.vTargetY);
	if(targetSpeed > 0.0f){
		positionController.targetX += ctrl.vTargetX * dt;
		positionController.targetY += ctrl.vTargetY * dt;

		/* a moving target stays within reach of the drone */
		float leadX = positionController.targetX - pos.x;
		float leadY = positionController.targetY - pos.y;
		PositionController_Limit(&leadX, &leadY, POS_CTRL_TARGET_LEAD_S * targetSpeed);
		positionController.targetX = pos.x + leadX;
		positionController.targetY = pos.y + leadY;
		ctrl.stopping = 1;
		ctrl.stopTime = 0.0f;
	}
	else if(ctrl.stopping){
		/* the target has stopped, the drone may not have yet (it is a little
		 * late, more so when the flow reads the speed too low): the target
		 * waits on the drone until that is at rest too, so it is not pulled back */
		ctrl.stopTime += dt;
		float speed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
		if(speed < POS_CTRL_STOPPED_VEL || ctrl.stopTime > POS_CTRL_BRAKE_MAX_S) ctrl.stopping = 0;
		positionController.targetX = pos.x;
		positionController.targetY = pos.y;
	}

	/* position error -> a speed, on top of the target's own */
	float vxSp = k->posKp * (positionController.targetX - pos.x);
	float vySp = k->posKp * (positionController.targetY - pos.y);
	PositionController_Limit(&vxSp, &vySp, POS_CTRL_VEL_MAX);
	vxSp += ctrl.vTargetX;
	vySp += ctrl.vTargetY;
	positionController.vxSetpoint = vxSp;
	positionController.vySetpoint = vySp;

	/* speed PI -> acceleration, level frame */
	float ex = vxSp - pos.vx;
	float ey = vySp - pos.vy;
	ctrl.iAccelX = PositionController_Clamp(ctrl.iAccelX + k->velKi * ex * dt, k->iMax);
	ctrl.iAccelY = PositionController_Clamp(ctrl.iAccelY + k->velKi * ey * dt, k->iMax);

	float ax = k->velKp * ex + ctrl.iAccelX + axTarget;
	float ay = k->velKp * ey + ctrl.iAccelY + ayTarget;
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
