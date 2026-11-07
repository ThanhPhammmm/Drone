#include "position_estimator_task.h"

#define POS_EST_PERIOD_S            0.01f     /* run the filter on every 10 ms of samples */
#define POS_EST_GAP_MAX_S           0.05f     /* longer gap between samples: start the window over */
#define POS_EST_LOSS_CYCLES         50U       /* 0.5 s with no flow velocity: not valid */

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

PositionEstimator_Handle_t positionEstimator;

static PosKF_t kfX;
static PosKF_t kfY;

typedef struct{
	uint8_t  haveSample;
	uint32_t lastSampleUs;
	float    dtAcc;
	float    accelXAcc;
	float    accelYAcc;

	float    rollTurn;          /* rad the body rolled since the last flow sample */
	float    pitchTurn;         /* rad it pitched */
	uint32_t lastFlowUs;

	uint8_t  flowPending;       /* a flow velocity waits for the next filter step */
	float    flowVx;            /* m/s, level frame */
	float    flowVy;
	float    flowStd;

	uint16_t usedAgo;           /* filter steps since the filter took a flow velocity */
	uint8_t  rejects;
	Altitude_Data_t alt;        /* last copy: is it standing on the floor? */
} PosEst_t;

static PosEst_t est;

void PositionEstimator_SetTaskHandle(TaskHandle_t handle){
	positionEstimator.positionTask = handle;
}

static void PositionEstimator_Init(void){
	memset(&est, 0, sizeof(est));
	est.usedAgo = POS_EST_LOSS_CYCLES;
	PosKF_Init(&kfX, 0.0f);
	PosKF_Init(&kfY, 0.0f);
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
	pos->valid        = airborne && (est.usedAgo < POS_EST_LOSS_CYCLES);
	pos->timestamp_us = att->timestamp_us;

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
