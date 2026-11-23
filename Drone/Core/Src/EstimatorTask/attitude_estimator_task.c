#include "attitude_topic.h"
#include "debug.h"
#include <math.h>

#define MAG_MAX_AGE_US		60000	/* the compass is read at ~50 Hz: older than this = stale */
#define MAG_INTERVAL_MAX_S	0.06f	/* the most time one reading stands for (after a gap: no catching up) */
#define MAG_TURN_TOL_DEG	3.0f	/* from one reading to the next the compass must turn as the gyro did */
#define MAG_TURN_MAX_S		0.1f	/* readings further apart are not compared */
#define MAG_REF_LPF			0.02f	/* per new reading, motors off: the reference follows over ~1 s */
#define MAG_NORM_MIN_G		0.15f	/* the earth's field is 0.25..0.65 gauss anywhere: outside this band */
#define MAG_NORM_MAX_G		1.00f	/* it is a bad calibration or a magnet, never learnt or used */
#define ATT_DEG2RAD			0.01745329f
#define ATT_PI				3.14159265f

#define MAHONY_KP_GROUND	1.0f
#define MAHONY_KP_FLIGHT	0.2f
#define MAHONY_KI_GROUND	0.05f
#define MAHONY_KI_FLIGHT	0.0f

static Mahony_t mahony;
AttitudeEstimator_Handle_t attitudeEstimator;

/* compass: each new reading is checked once against a reference of the earth's
 * field here (strength and dip), learnt while the motors are off */
static struct{
	uint32_t lastUs;		/* the reading handled last */
	uint8_t fresh;			/* readings were arriving at the last step */
	uint32_t samples;		/* readings learnt into the reference */
	float refNorm;
	float refDip;			/* rad */
	uint8_t valid;			/* the last reading gave a heading */
	uint8_t ok;				/* ... and matched the reference */
	float heading;			/* rad, its compass heading */
	float gyroTurn;			/* rad, the heading change the gyro alone sees, summed up */
	float lastTurn;			/* gyroTurn at the last reading */
	uint8_t haveLast;		/* the last reading gave a heading too */
} magChk;

static float AttitudeEstimator_Wrap(float a){
	while(a > ATT_PI) a -= 2.0f * ATT_PI;
	while(a < -ATT_PI) a += 2.0f * ATT_PI;
	return a;
}

void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	attitudeEstimator.attitudeTask = handle;
}

void AttitudeEstimatorTask(void *argument){
	AttitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	IMUTopic_Subscribe(attitudeEstimator.attitudeTask, ATTITUDE_ESTIMATOR_ID_TASK);

	BMI088_Data_t imu;
	Mahony_Init(&mahony, MAHONY_KP_GROUND, MAHONY_KI_GROUND);
	mahony.declination = ATT_EST_MAG_DECLINATION_DEG * ATT_DEG2RAD;
	mahony.magSnapRad = ATT_EST_MAG_SNAP_DEG * ATT_DEG2RAD;
	
	while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(IMUTopic_Copy(&imu) != pdPASS) continue;

		float ax = imu.accel.x;
		float ay = imu.accel.y;
		float az = imu.accel.z;

		uint8_t motors = Arm_MotorsAllowed();

		/* the gyro's own heading change (about the earth vertical), to check the compass against.
		 * A field fixed to the frame (motor current) turns with the drone; the earth's does not */
		if(imu.dt > 0.0f && imu.dt < 0.05f){
			float q0 = mahony.q[0], q1 = mahony.q[1], q2 = mahony.q[2], q3 = mahony.q[3];
			float down = 2.0f * (q1*q3 - q0*q2) * (imu.gyro.x + mahony.bias[0]) +
						2.0f * (q0*q1 + q2*q3) * (imu.gyro.y + mahony.bias[1]) +
						(q0*q0 - q1*q1 - q2*q2 + q3*q3) * (imu.gyro.z + mahony.bias[2]);
			magChk.gyroTurn = AttitudeEstimator_Wrap(magChk.gyroTurn + down * imu.dt);
		}

		/* ---- compass ---- */
		Mag_Data_t mag = {0};
		uint8_t magFresh = 0;
		if(MagTopic_Copy(&mag, 0) == pdPASS && mag.timestamp_us != 0){
			int32_t age_us = Time_DiffUs(imu.timestamp_us, mag.timestamp_us);
			if(age_us < MAG_MAX_AGE_US && age_us > -MAG_MAX_AGE_US){
				magFresh = 1;
			}
		}
		float magVec[3] = { mag.x, mag.y, mag.z };

		uint8_t magNew = 0;
		float magInterval = 0.0f;	/* s: the time this reading stands for */
		float magGap = 1.0f;	/* s since the last reading */
		if(magFresh && mag.timestamp_us != magChk.lastUs){
			magNew = 1;
			if(magChk.lastUs != 0){
				magGap = (float)(uint32_t)(mag.timestamp_us - magChk.lastUs) * 1e-6f;
				magInterval = (magGap < MAG_INTERVAL_MAX_S) ? magGap : MAG_INTERVAL_MAX_S;
			}
			/* readings came back after a gap with the motors off (a compass calibration
			 * ends like that): the field may read differently now, learn it again */
			if(!magChk.fresh && !motors) magChk.samples = 0;
			magChk.lastUs = mag.timestamp_us;

			float headingErr, dip, norm;
			magChk.valid = Mahony_MagGeometry(&mahony, magVec, &headingErr, &dip, &norm);
			magChk.ok = 0;
			if(magChk.valid){
				uint8_t earthLike = norm > MAG_NORM_MIN_G && norm < MAG_NORM_MAX_G;
				if(!motors && earthLike){
					/* motors off: this is the earth's field here */
					if(magChk.samples == 0){
						magChk.refNorm = norm;
						magChk.refDip = dip;
					}
					else{
						magChk.refNorm += (norm - magChk.refNorm) * MAG_REF_LPF;
						magChk.refDip += (dip - magChk.refDip) * MAG_REF_LPF;
					}
					if(magChk.samples < ATT_EST_MAG_REF_SAMPLES) magChk.samples++;
				}
				/* the heading the compass alone gives (for the TFT), even when it is not used */
				float r, p, yawNow;
				Mahony_GetEuler(&mahony, &r, &p, &yawNow);
				float heading = AttitudeEstimator_Wrap(yawNow - headingErr);

				/* since the last reading the compass turned as much as the gyro did */
				uint8_t turnOk = magChk.haveLast && magGap <= MAG_TURN_MAX_S &&
								 fabsf(AttitudeEstimator_Wrap((heading - magChk.heading) -
															(magChk.gyroTurn - magChk.lastTurn)))
									 < MAG_TURN_TOL_DEG * ATT_DEG2RAD;
				magChk.heading = heading;
				magChk.lastTurn = magChk.gyroTurn;

				/* motor current bends the field: its strength or dip changes, or it turns with the frame */
				magChk.ok = earthLike && turnOk && magChk.samples >= ATT_EST_MAG_REF_SAMPLES &&
							fabsf(norm - magChk.refNorm) < ATT_EST_MAG_NORM_TOL * magChk.refNorm &&
							fabsf(dip - magChk.refDip) < ATT_EST_MAG_DIP_TOL_DEG * ATT_DEG2RAD;
			}
			magChk.haveLast = magChk.valid;
		}
		magChk.fresh = magFresh;
		uint8_t magValid = magFresh && magChk.valid;
		uint8_t magOk = magValid && magChk.ok;

		mahony.kp		= motors ? MAHONY_KP_FLIGHT : MAHONY_KP_GROUND;
		mahony.ki		= motors ? MAHONY_KI_FLIGHT : MAHONY_KI_GROUND;
		mahony.magKp	= motors ? ATT_EST_MAG_KP_FLIGHT : ATT_EST_MAG_KP_GROUND;
		mahony.magKi	= motors ? ATT_EST_MAG_KI_FLIGHT : ATT_EST_MAG_KI_GROUND;
		mahony.magSnap = !motors;	/* never jump the heading in the air */
		mahony.magInterval = magInterval;
		/* each reading is used once, on the step it arrives: never compared later with a newer heading */
		Mahony_Update(&mahony, &imu, magVec, ATT_EST_USE_MAG && magOk && magNew && magInterval > 0.0f);

		Attitude_Data_t *attitude = &attitudeEstimator.data;
		attitude->magHeading = magChk.heading;
		attitude->magStatus = (magValid ? ATT_MAG_FRESH : 0) | (magOk ? ATT_MAG_OK : 0) |
							((ATT_EST_USE_MAG && mahony.headingAligned) ? ATT_MAG_IN_USE : 0) |
							((magChk.samples >= ATT_EST_MAG_REF_SAMPLES) ? ATT_MAG_REF : 0);

		Mahony_GetQuaternion(&mahony, attitude->q);
		attitude->q0 = attitude->q[0];
		attitude->q1 = attitude->q[1];
		attitude->q2 = attitude->q[2];
		attitude->q3 = attitude->q[3];

		Mahony_GetEuler(&mahony, &attitude->roll, &attitude->pitch, &attitude->yaw);

		float rate[3];
		Mahony_GetRate(&mahony, rate);

		attitude->rollRate	= rate[0];
		attitude->pitchRate	= rate[1];
		attitude->yawRate	= rate[2];

		attitude->gyroBiasX = mahony.bias[0];
		attitude->gyroBiasY = mahony.bias[1];
		attitude->gyroBiasZ = mahony.bias[2];

		float q0 = attitude->q0; 
		float q1 = attitude->q1;
		float q2 = attitude->q2;
		float q3 = attitude->q3;

		// altitude accelerometer calculation
		// DCM(Direction Cosine Matrix)
		/*
			Body frame and earth frame:
				Body frame is fixed to the drone: 
					x = nose (front), 
					y = right, 
					z = down.
				Earth frame is fixed to the room: 
					x = where the nose pointed at power-up, 
					y = to its right, 
					z = straight down.

			earth = R × body

			
						| cos(theta)cos(psi)	sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi)	cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi)  	|
			Reuler  =  	| cos(theta)sin(psi)	sin(phi)sin(theta)sin(psi) + cos(phi)cos(psi)	cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi)  	|
						| -sin(theta)			sin(phi)cos(theta)								cos(phi)cos(theta) 								| 		


								| q0²+q1²−q2²−q3²    2(q1q2 − q0q3)     2(q1q3 + q0q2)  |
			Rquaternion  	=  	| 2(q1q2 + q0q3)     q0²−q1²+q2²−q3²    2(q2q3 − q0q1)  |
								| 2(q1q3 − q0q2)     2(q2q3 + q0q1)     q0²−q1²−q2²+q3² |

			A column is one body axis, as seen in the room.

				Column				Body axis		Meaning
				1 (R11, R21, R31)	nose 			where the nose points in the room
				2 (R12, R22, R32)	right 			arm where the right side points
				3 (R13, R23, R33)	body 			down where the drone’s belly points (thrust goes the other way)

			A row is one room axis, as seen by the drone.

				Row		Room 	axis
				1		earth 	x
				2		earth 	y
				3		earth 	down
		*/

		/* the third DCM rows: vertical acceleration */
		float rx = 2.0f * (q1*q3 - q0*q2);
		float ry = 2.0f * (q0*q1 + q2*q3);
		float rz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

		float fDown = rx*ax + ry*ay + rz*az;

		attitude->accelUp = -(fDown + BMI088_G);

		/* the first two DCM rows: horizontal acceleration */
		attitude->accelX = (q0*q0 + q1*q1 - q2*q2 - q3*q3) * ax
						 + 2.0f * (q1*q2 - q0*q3) * ay
						 + 2.0f * (q1*q3 + q0*q2) * az;
		attitude->accelY = 2.0f * (q1*q2 + q0*q3) * ax
						 + (q0*q0 - q1*q1 + q2*q2 - q3*q3) * ay
						 + 2.0f * (q2*q3 - q0*q1) * az;

		attitude->dt = imu.dt;
		attitude->timestamp_us = imu.timestamp_us;

		AttitudeTopic_Publish(attitude);
	}
}
