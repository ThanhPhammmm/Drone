#include "position_estimator_task.h"

#define POS_EST_PERIOD_S            0.01f     /* run the filter on every 10 ms of samples */
#define POS_EST_GAP_MAX_S           0.05f     /* longer gap between samples: start the window over */
#define POS_EST_LOSS_CYCLES         50U       /* 0.5 s with no flow velocity: the flow is not in use */

#define POS_EST_FLOW_SWAP_XY        0         /* 0: sensor X is the nose axis, 1: sensor Y is */
#define POS_EST_FLOW_FWD_SIGN       (1.0f)
#define POS_EST_FLOW_RIGHT_SIGN     (1.0f)

#define POS_EST_FLOW_MAX_AGE_US     60000
#define POS_EST_FLOW_DT_MIN_US      5000
#define POS_EST_FLOW_DT_MAX_US      150000

#define POS_EST_HEIGHT_MIN_M        0.12f     /* the PMW3901 lens needs 80 mm to the floor */
#define POS_EST_HEIGHT_MAX_M        2.5f      /* higher: few counts per sample, and a noisier ToF */
#define POS_EST_TILT_COS            0.82f     /* more than 35 deg of tilt: the two sensors look too far apart */

#define POS_EST_FLOW_STD_MPS        0.05f     /* 1 sigma the filter gives a flow velocity... */
#define POS_EST_FLOW_STD_PER_M      0.10f     /* ...plus this per metre of height */
#define POS_EST_FLOW_GATE           9.0f      /* innovation^2 / S: 3 sigma */
#define POS_EST_REJECTS_MAX         5U        /* outliers in a row: the filter is off, start over from the flow */
#define POS_EST_GROUND_V_STD        0.05f     /* standing on the floor: v = 0, which lets the filter learn the accel bias */

/* GPS (NEO-M8N), only when its fix is good. Without the flow (higher than
 * POS_EST_HEIGHT_MAX_M, or over a floor the flow cannot see) it measures
 * velocity and position. With the flow it only pulls the flow's slow drift
 * back, once the gap is bigger than the GPS's own accuracy. Its north/east must be turned into the
 * level frame, so the angle between the two is learned first: while the flow
 * is in use and the drone moves, the direction the GPS says it moves is
 * compared with the direction the flow says. No compass needed. */
#define POS_EST_GPS_MAX_AGE_US      300000    /* a solution older than this: not used */
#define POS_EST_GPS_DELAY_STEPS     15U       /* 150 ms: a solution describes the drone this long ago */
#define POS_EST_GPS_SATS_MIN        6U
#define POS_EST_GPS_HACC_MAX        3.0f      /* m, worse: not used */
#define POS_EST_GPS_SACC_MAX        0.6f      /* m/s, worse: not used */
#define POS_EST_GPS_POS_STD_MIN     1.0f      /* m: never trusted closer than this */
#define POS_EST_GPS_FLOW_POS_SCALE  2.0f      /* with the flow: GPS position trusted 2x less, it pulls the drift back slowly */
#define POS_EST_GPS_FLOW_DEADBAND   1.0f      /* with the flow: x hAcc. Closer than this to the GPS: the GPS wander, not drift, left alone */
#define POS_EST_GPS_VEL_STD_MIN     0.15f     /* m/s */
#define POS_EST_GPS_POS_GATE        25.0f     /* innovation^2 / S: 5 sigma */
#define POS_EST_GPS_VEL_GATE        9.0f      /* 3 sigma */
#define POS_EST_GPS_REJECTS_MAX     5U        /* outliers in a row: start over from the GPS */
#define POS_EST_GPS_LOSS_CYCLES     100U      /* 1 s with no GPS update: the GPS is not in use */
#define POS_EST_GPS_TAKEOVER_CYCLES 20U       /* 0.2 s with no flow: the GPS takes over (before the flow's 0.5 s run out) */
#define POS_EST_GPS_REANCHOR_CYCLES 100U      /* GPS position unused 1 s: tie it to the estimate again on return */

#define POS_EST_ALIGN_SPEED         0.8f      /* m/s the flow and the GPS must both see */
#define POS_EST_ALIGN_SAMPLES       10U       /* consistent solutions (2 s at 5 Hz) before the angle is trusted */
#define POS_EST_ALIGN_SPREAD        0.97f     /* length of the mean direction: samples within ~14 deg */
#define POS_EST_ALIGN_TRACK_GAIN    0.05f     /* after that: follow the slow gyro heading drift */

#define POS_EST_HISTORY             32U       /* filter steps kept for the GPS delay */
#define POS_EST_EARTH_RADIUS_M      6371000.0f
#define POS_EST_E7DEG_TO_RAD        (1e-7f * 0.017453293f)
#define POS_EST_PI                  3.14159265f

_Static_assert(POS_EST_HISTORY > POS_EST_GPS_DELAY_STEPS, "the history must cover the GPS delay");

PositionEstimator_Handle_t positionEstimator;

static PosKF_t kfX;
static PosKF_t kfY;

static PosEst_t  est;
static PosGps_t  gps;
static PosHist_t hist[POS_EST_HISTORY];
static uint8_t   histIdx;           /* the next slot to write */

void PositionEstimator_SetTaskHandle(TaskHandle_t handle){
	positionEstimator.positionTask = handle;
}

static void PositionEstimator_Init(void){
	memset(&est, 0, sizeof(est));
	memset(&gps, 0, sizeof(gps));
	memset(hist, 0, sizeof(hist));
	histIdx = 0;
	est.usedAgo    = POS_EST_LOSS_CYCLES;
	gps.usedAgo    = POS_EST_GPS_LOSS_CYCLES;
	gps.posUsedAgo = POS_EST_GPS_REANCHOR_CYCLES;
#if ATT_EST_USE_MAG
	gps.aligned    = 1;             /* yaw 0 is (magnetic) north already */
#endif
	PosKF_Init(&kfX, 0.0f);
	PosKF_Init(&kfY, 0.0f);
}

static float PositionEstimator_Wrap(float a){
	while(a >  POS_EST_PI) a -= 2.0f * POS_EST_PI;
	while(a < -POS_EST_PI) a += 2.0f * POS_EST_PI;
	return a;
}

/* Flow sample -> velocity in the level frame. 0 = it cannot be used.
 * The floor seems to move by (v / d) dt, d = distance along the lens axis,
 * plus the body's own turning: pitching nose up looks like flying forward,
 * rolling right like flying left. Those turns come from the gyro. */
static uint8_t PositionEstimator_FlowVelocity(const Attitude_Data_t *att, const Flow_Data_t *flow,
                                             float rollTurn, float pitchTurn, float *vx, float *vy, float *std){
	if(!flow->valid) return 0;
	if(flow->dt_us < POS_EST_FLOW_DT_MIN_US || flow->dt_us > POS_EST_FLOW_DT_MAX_US) return 0;
	int32_t age = Time_DiffUs(att->timestamp_us, flow->timestamp_us);
	if(age >= POS_EST_FLOW_MAX_AGE_US || age <= -POS_EST_FLOW_MAX_AGE_US) return 0;

	Altitude_Data_t alt;
	if(AltitudeTopic_Copy(&alt, 0) != pdPASS || !alt.aglValid) return 0;
	if(alt.agl < POS_EST_HEIGHT_MIN_M || alt.agl > POS_EST_HEIGHT_MAX_M) return 0;

	/* body z seen from the earth frame: cos(roll) * cos(pitch) */
	float tiltCos = att->q0*att->q0 - att->q1*att->q1 - att->q2*att->q2 + att->q3*att->q3;
	if(tiltCos < POS_EST_TILT_COS) return 0;
	float d = alt.agl / tiltCos;

#if POS_EST_FLOW_SWAP_XY
	float countsFwd   = (float)flow->deltaY;
	float countsRight = (float)flow->deltaX;
#else
	float countsFwd   = (float)flow->deltaX;
	float countsRight = (float)flow->deltaY;
#endif
	float angleFwd   = POS_EST_FLOW_FWD_SIGN   * countsFwd   * PMW3901_RAD_PER_COUNT - pitchTurn;
	float angleRight = POS_EST_FLOW_RIGHT_SIGN * countsRight * PMW3901_RAD_PER_COUNT + rollTurn;

	float dt     = (float)flow->dt_us * 1e-6f;
	float vFwd   = angleFwd   * d / dt;
	float vRight = angleRight * d / dt;

	/* heading frame -> level frame */
	float c = cosf(att->yaw);
	float s = sinf(att->yaw);
	*vx  = c * vFwd - s * vRight;
	*vy  = s * vFwd + c * vRight;
	*std = POS_EST_FLOW_STD_MPS + POS_EST_FLOW_STD_PER_M * d;
	return 1;
}

/* While the flow measures the velocity and the drone moves: the GPS velocity
 * points in north/east, the estimate's in the level frame. The angle between
 * the two is the heading offset. Averaged over a few solutions first, then
 * followed slowly (the gyro heading drifts ~2 deg/min). */
static void PositionEstimator_Align(const Gps_Data_t *g, const PosHist_t *then, uint8_t flowFresh){
	if(!flowFresh) return;
	float gpsSpeed = sqrtf(g->velN * g->velN + g->velE * g->velE);
	float estSpeed = sqrtf(then->vx * then->vx + then->vy * then->vy);
	if(gpsSpeed < POS_EST_ALIGN_SPEED || estSpeed < POS_EST_ALIGN_SPEED) return;

	float diff = PositionEstimator_Wrap(atan2f(g->velE, g->velN) - atan2f(then->vy, then->vx));
	if(gps.aligned){
		gps.yawOffset = PositionEstimator_Wrap(gps.yawOffset +
		                POS_EST_ALIGN_TRACK_GAIN * PositionEstimator_Wrap(diff - gps.yawOffset));
		return;
	}

	float w = (gpsSpeed < estSpeed) ? gpsSpeed : estSpeed;
	gps.alignC += w * cosf(diff);
	gps.alignS += w * sinf(diff);
	gps.alignW += w;
	if(++gps.alignN < POS_EST_ALIGN_SAMPLES) return;

	float spread = sqrtf(gps.alignC * gps.alignC + gps.alignS * gps.alignS) / gps.alignW;
	if(spread >= POS_EST_ALIGN_SPREAD){
		gps.yawOffset = atan2f(gps.alignS, gps.alignC);
		gps.aligned   = 1;
	}
	else{
		/* the samples disagree (sharp turns, gusts): keep half, look again */
		gps.alignC *= 0.5f;
		gps.alignS *= 0.5f;
		gps.alignW *= 0.5f;
		gps.alignN  = POS_EST_ALIGN_SAMPLES / 2U;
	}
}

/* once per filter step: a new GPS solution, if one came. flowFresh: the flow
 * measured the velocity within POS_EST_GPS_TAKEOVER_CYCLES */
static void PositionEstimator_Gps(uint32_t nowUs, uint8_t standing, uint8_t flowFresh){
	if(gps.usedAgo    < POS_EST_GPS_LOSS_CYCLES)     gps.usedAgo++;
	if(gps.posUsedAgo < POS_EST_GPS_REANCHOR_CYCLES) gps.posUsedAgo++;

	Gps_Data_t g;
	if(GpsTopic_Copy(&g, 0) != pdPASS || g.timestamp_us == 0 || g.timestamp_us == gps.lastStampUs) return;
	gps.lastStampUs = g.timestamp_us;

	int32_t age = Time_DiffUs(nowUs, g.timestamp_us);
	if(age > POS_EST_GPS_MAX_AGE_US || age < -POS_EST_GPS_MAX_AGE_US) return;
	if(!g.fixOk || g.fixType < 3 || g.numSV < POS_EST_GPS_SATS_MIN) return;
	if(g.hAcc > POS_EST_GPS_HACC_MAX || g.sAcc > POS_EST_GPS_SACC_MAX) return;
	if(est.steps < POS_EST_HISTORY) return;            /* the delay needs a full history */

	/* north/east from the first good fix (flat earth: fine over a few km) */
	if(!gps.originSet){
		gps.lat0      = g.lat;
		gps.lon0      = g.lon;
		gps.cosLat0   = cosf((float)g.lat * POS_EST_E7DEG_TO_RAD);
		gps.originSet = 1;
	}
	float north = (float)(g.lat - gps.lat0) * POS_EST_E7DEG_TO_RAD * POS_EST_EARTH_RADIUS_M;
	float east  = (float)(g.lon - gps.lon0) * POS_EST_E7DEG_TO_RAD * POS_EST_EARTH_RADIUS_M * gps.cosLat0;

	/* the estimate as it was when the receiver measured */
	const PosHist_t *then = &hist[(histIdx + POS_EST_HISTORY - POS_EST_GPS_DELAY_STEPS) % POS_EST_HISTORY];

	PositionEstimator_Align(&g, then, flowFresh);
	if(!gps.aligned) return;

	/* north/east -> level frame: turn back by the offset */
	float c   = cosf(gps.yawOffset);
	float s   = sinf(gps.yawOffset);
	float gx  =  c * north  + s * east;
	float gy  = -s * north  + c * east;
	float gvx =  c * g.velN + s * g.velE;
	float gvy = -s * g.velN + c * g.velE;

	/* Only a good fix gets here (checked above): a bad one is ignored.
	 * Position: always, so it also stops the flow's slow drift.
	 * Velocity: only without the flow, which measures it better. */
	uint8_t posUse = !standing;
	uint8_t velUse = !standing && !flowFresh;
	if(posUse && gps.posUsedAgo >= POS_EST_GPS_REANCHOR_CYCLES){
		/* first use, or back after a break: put the fix, as it was then, on
		 * the estimate as it was then. No jump: from here on the GPS only
		 * stops the drift. */
		gps.originX = then->x - gx;
		gps.originY = then->y - gy;
	}
	gps.x = gx + gps.originX;
	gps.y = gy + gps.originY;

	if(!posUse) return;

	/* measured then, applied now: shifted by how far the estimate moved since */
	if(velUse){
		float vx = PosKF_Get(&kfX, POS_KF_V);
		float vy = PosKF_Get(&kfY, POS_KF_V);
		float velStd = (g.sAcc > POS_EST_GPS_VEL_STD_MIN) ? g.sAcc : POS_EST_GPS_VEL_STD_MIN;
		float velX   = gvx + (vx - then->vx);
		float velY   = gvy + (vy - then->vy);
		uint8_t velUsedX = PosKF_UpdateVelocity(&kfX, velX, velStd, POS_EST_GPS_VEL_GATE);
		uint8_t velUsedY = PosKF_UpdateVelocity(&kfY, velY, velStd, POS_EST_GPS_VEL_GATE);
		if(velUsedX && velUsedY){
			gps.velRejects = 0;
			gps.usedAgo    = 0;
		}
		else if(++gps.velRejects >= POS_EST_GPS_REJECTS_MAX){
			PosKF_SetVelocity(&kfX, velX, velStd);
			PosKF_SetVelocity(&kfY, velY, velStd);
			gps.velRejects = 0;
			gps.usedAgo    = 0;
		}
	}

	float px = PosKF_Get(&kfX, POS_KF_P);
	float py = PosKF_Get(&kfY, POS_KF_P);
	float posStd = (g.hAcc > POS_EST_GPS_POS_STD_MIN) ? g.hAcc : POS_EST_GPS_POS_STD_MIN;
	float zx = gps.x + (px - then->x);
	float zy = gps.y + (py - then->y);
	if(flowFresh){
		/* The flow holds the spot better than the GPS, which wanders about
		 * its hAcc. Only the gap beyond that is drift: pull just that back. */
		float ix   = gps.x - then->x;
		float iy   = gps.y - then->y;
		float gap  = sqrtf(ix * ix + iy * iy);
		float band = POS_EST_GPS_FLOW_DEADBAND * posStd;
		if(gap <= band){
			gps.posRejects = 0;
			gps.posUsedAgo = 0;     /* in use: no new anchor */
			gps.usedAgo    = 0;
			return;
		}
		float keep = (gap - band) / gap;
		zx = px + ix * keep;
		zy = py + iy * keep;
		posStd *= POS_EST_GPS_FLOW_POS_SCALE;
	}
	uint8_t posUsedX = PosKF_UpdatePosition(&kfX, zx, posStd, POS_EST_GPS_POS_GATE);
	uint8_t posUsedY = PosKF_UpdatePosition(&kfY, zy, posStd, POS_EST_GPS_POS_GATE);
	if(posUsedX && posUsedY){
		gps.posRejects = 0;
		gps.posUsedAgo = 0;
		gps.usedAgo    = 0;
	}
	else if(++gps.posRejects >= POS_EST_GPS_REJECTS_MAX){
		/* the fix jumped: tie it to the estimate again with the next one */
		gps.posRejects = 0;
		gps.posUsedAgo = POS_EST_GPS_REANCHOR_CYCLES;
	}
}

/* one attitude sample (1 kHz) */
static void PositionEstimator_Step(const Attitude_Data_t *att){
	if(!est.haveSample){
		est.lastSampleUs = att->timestamp_us;
		est.haveSample   = 1;
		return;
	}

	if(att->timestamp_us == est.lastSampleUs) return;
	float sampleDt = (float)(uint32_t)(att->timestamp_us - est.lastSampleUs) * 1e-6f;
	est.lastSampleUs = att->timestamp_us;

	if(sampleDt > POS_EST_GAP_MAX_S){
		est.dtAcc     = 0.0f;
		est.accelXAcc = 0.0f;
		est.accelYAcc = 0.0f;
		est.rollTurn  = 0.0f;
		est.pitchTurn = 0.0f;
		return;
	}

	est.rollTurn  += att->rollRate  * sampleDt;
	est.pitchTurn += att->pitchRate * sampleDt;

	/* a new flow sample: make it a velocity now, while the turns above cover the same time */
	Flow_Data_t flow;
	if(FlowTopic_Copy(&flow, 0) == pdPASS && flow.timestamp_us != 0 && flow.timestamp_us != est.lastFlowUs){
		est.lastFlowUs  = flow.timestamp_us;
		est.flowPending = PositionEstimator_FlowVelocity(att, &flow, est.rollTurn, est.pitchTurn,
		                                                 &est.flowVx, &est.flowVy, &est.flowStd);
		est.rollTurn  = 0.0f;
		est.pitchTurn = 0.0f;
	}

	est.dtAcc     += sampleDt;
	est.accelXAcc += att->accelX * sampleDt;
	est.accelYAcc += att->accelY * sampleDt;
	if(est.dtAcc < POS_EST_PERIOD_S) return;

	float dt = est.dtAcc;
	float ax = est.accelXAcc / est.dtAcc;
	float ay = est.accelYAcc / est.dtAcc;
	est.dtAcc     = 0.0f;
	est.accelXAcc = 0.0f;
	est.accelYAcc = 0.0f;

	PosKF_Predict(&kfX, ax, dt);
	PosKF_Predict(&kfY, ay, dt);

	if(est.usedAgo < POS_EST_LOSS_CYCLES) est.usedAgo++;

	/* Not flying and standing on the floor (or too low for the flow to see):
	 * it does not move, and telling the filter so lets it learn the accel bias. */
	uint8_t airborne = Arm_IsAirborne();
	(void)AltitudeTopic_Copy(&est.alt, 0);
	uint8_t onFloor  = Arm_MotorsAllowed() && est.alt.onFloor;
	uint8_t standing = !airborne && (onFloor || !est.alt.aglValid || est.alt.agl < POS_EST_HEIGHT_MIN_M);
	if(standing){
		PosKF_UpdateVelocity(&kfX, 0.0f, POS_EST_GROUND_V_STD, 0.0f);
		PosKF_UpdateVelocity(&kfY, 0.0f, POS_EST_GROUND_V_STD, 0.0f);
		est.flowPending = 0;
		est.rejects     = 0;
	}
	else if(est.flowPending){
		est.flowPending = 0;
		uint8_t usedX = PosKF_UpdateVelocity(&kfX, est.flowVx, est.flowStd, POS_EST_FLOW_GATE);
		uint8_t usedY = PosKF_UpdateVelocity(&kfY, est.flowVy, est.flowStd, POS_EST_FLOW_GATE);
		if(usedX && usedY){
			est.rejects = 0;
			est.usedAgo = 0;
		}
		else if(++est.rejects >= POS_EST_REJECTS_MAX){
			/* not a glitch, it keeps disagreeing: the filter is off, start over from the flow */
			PosKF_SetVelocity(&kfX, est.flowVx, est.flowStd);
			PosKF_SetVelocity(&kfY, est.flowVy, est.flowStd);
			est.rejects = 0;
			est.usedAgo = 0;
		}
	}

	uint8_t flowFresh = (est.usedAgo < POS_EST_GPS_TAKEOVER_CYCLES);
	uint8_t flowInUse = (est.usedAgo < POS_EST_LOSS_CYCLES);
	PositionEstimator_Gps(att->timestamp_us, standing, flowFresh);
	uint8_t gpsInUse  = (gps.usedAgo < POS_EST_GPS_LOSS_CYCLES);

	float vx = PosKF_Get(&kfX, POS_KF_V);
	float vy = PosKF_Get(&kfY, POS_KF_V);
	float c  = cosf(att->yaw);
	float s  = sinf(att->yaw);

	Position_Data_t *pos = &positionEstimator.data;
	pos->x            = PosKF_Get(&kfX, POS_KF_P);
	pos->y            = PosKF_Get(&kfY, POS_KF_P);
	pos->vx           = vx;
	pos->vy           = vy;
	pos->vFwd         =  c * vx + s * vy;
	pos->vRight       = -s * vx + c * vy;
	pos->yaw          = att->yaw;
	pos->valid        = airborne && (flowInUse || gpsInUse);
	pos->source       = (gpsInUse && !flowFresh) ? POS_SOURCE_GPS : (flowInUse ? POS_SOURCE_FLOW : POS_SOURCE_NONE);
	pos->gpsAligned   = gps.aligned;
	pos->gpsYawOffset = gps.yawOffset;
	pos->gpsX         = gps.aligned ? gps.x : 0.0f;
	pos->gpsY         = gps.aligned ? gps.y : 0.0f;
	pos->timestamp_us = att->timestamp_us;

	/* this step's estimate, for the GPS delay */
	hist[histIdx].x  = pos->x;
	hist[histIdx].y  = pos->y;
	hist[histIdx].vx = vx;
	hist[histIdx].vy = vy;
	histIdx = (uint8_t)((histIdx + 1U) % POS_EST_HISTORY);
	if(est.steps < POS_EST_HISTORY) est.steps++;

	PositionTopic_Publish(pos);
}

void PositionEstimatorTask(void *argument){
	PositionEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	AttitudeTopic_Subscribe(positionEstimator.positionTask, POSITION_ESTIMATOR_ID_TASK);

	PositionEstimator_Init();

	Attitude_Data_t attitude;

	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(AttitudeTopic_Copy(&attitude) != pdPASS) continue;

		PositionEstimator_Step(&attitude);
	}
}
