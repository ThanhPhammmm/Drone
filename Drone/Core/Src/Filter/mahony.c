#include "mahony.h"
#include <math.h>

#define MAHONY_G				9.80665f
#define MAHONY_ACC_TOL			0.20f	/* accept 0.8 g .. 1.2 g */

#define MAHONY_BIAS_LIMIT		0.10f	/* ~5.7 deg/s */

#define MAHONY_DT_MAX			0.05f

#define MAHONY_PI				3.14159265f
#define MAHONY_MAG_BIAS_ERR		0.10f	/* rad (~6 deg): the most heading error the bias learns from,
										* so a field bent for a while (steel nearby) teaches little drift */
#define MAHONY_MAG_ERR_MAX		0.26f	/* rad (~15 deg): the most heading error one reading corrects at
										* full gain. A bigger one (a bent field no check caught, or a real
										* error in flight) is pulled in slowly: 1.5 deg/s in flight */
#define MAHONY_MAG_MAX_RATE		1.0f	/* rad/s (~57 deg/s): turning faster, the compass is not used.
										* Its reading is up to ~10 ms old when it arrives: at a fast
										* turn that alone looks like a heading error */

static inline float mahony_clamp(float v, float lim){
	return v < -lim ? -lim : (v > lim ? lim : v);
}

static inline float mahony_wrap_pi(float a){
	while(a > MAHONY_PI) a -= 2.0f * MAHONY_PI;
	while(a < -MAHONY_PI) a += 2.0f * MAHONY_PI;
	return a;
}

static inline uint8_t normalize3(float *x, float *y,float *z){
	float norm = sqrtf((*x)*(*x) + (*y)*(*y) + (*z)*(*z));
	if(norm < 1e-6f) return 0;

	norm = 1.0f / norm;

	*x *= norm;
	*y *= norm;
	*z *= norm;

	return 1;
}

/* Rotate the quaternion about the earth vertical by `dyaw` (rad). */
static void mahony_rotate_yaw(Mahony_t *m, float dyaw){
	float c = cosf(0.5f * dyaw);
	float s = sinf(0.5f * dyaw);
	float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];

	/* q' = (c, 0, 0, s) * q  -- earth-frame rotation */
	m->q[0] = c*q0 - s*q3;
	m->q[1] = c*q1 - s*q2;
	m->q[2] = c*q2 + s*q1;
	m->q[3] = c*q3 + s*q0;
}

void Mahony_Init(Mahony_t *m, float kp,float ki){
	m->kp = kp;
	m->ki = ki;

	m->q[0] = 1.0f;
	m->q[1] = 0.0f;
	m->q[2] = 0.0f;
	m->q[3] = 0.0f;

	m->bias[0] = 0.0f;
	m->bias[1] = 0.0f;
	m->bias[2] = 0.0f;

	m->rate[0] = 0.0f;
	m->rate[1] = 0.0f;
	m->rate[2] = 0.0f;

	m->magKp			= 0.0f;
	m->magKi			= 0.0f;
	m->declination		= 0.0f;
	m->magSnap			= 0;
	m->magSnapRad		= MAHONY_PI;
	m->magInterval		= 0.0f;
	m->headingAligned	= 0;
}

uint8_t Mahony_MagGeometry(const Mahony_t *m, const float mag[3], float *headingErr, float *dip, float *norm){
	float mx = mag[0], my = mag[1], mz = mag[2];
	float n = sqrtf(mx*mx + my*my + mz*mz);
	if(!(n > 1e-6f && n < 1e6f)) return 0;
	mx /= n; my /= n; mz /= n;

	float q0 = m->q[0], q1 = m->q[1], q2 = m->q[2], q3 = m->q[3];
	/* measured field -> earth frame (rows of the body -> earth rotation) */
	float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2.0f * my * (q1*q2 - q0*q3) + 2.0f * mz * (q1*q3 + q0*q2);
	float hy = 2.0f * mx * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2.0f * mz * (q2*q3 - q0*q1);
	float hz = 2.0f * mx * (q1*q3 - q0*q2) + 2.0f * my * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
	if(!(hx*hx + hy*hy >= 1e-4f)) return 0;		/* field (almost) vertical: no heading in it */

	/* The error of the guess */
	if(headingErr) *headingErr = mahony_wrap_pi(atan2f(hy, hx) - m->declination);
	if(dip) *dip = asinf(hz > 1.0f ? 1.0f : (hz < -1.0f ? -1.0f : hz));
	if(norm) *norm = n;
	return 1;
}

void Mahony_Update(Mahony_t *m, const BMI088_Data_t *imu, const float mag[3], uint8_t magValid){
	float ax = imu->accel.x;
	float ay = imu->accel.y;
	float az = imu->accel.z;
	float dt = imu->dt;

	if(!(dt > 0.0f) || dt > MAHONY_DT_MAX){
		m->rate[0] = imu->gyro.x + m->bias[0];
		m->rate[1] = imu->gyro.y + m->bias[1];
		m->rate[2] = imu->gyro.z + m->bias[2];
		return;
	}

	float aNorm = sqrtf(ax*ax + ay*ay + az*az);
	uint8_t accValid = (aNorm > MAHONY_G * (1.0f - MAHONY_ACC_TOL)) && (aNorm < MAHONY_G * (1.0f + MAHONY_ACC_TOL));
	if(accValid) accValid = normalize3(&ax, &ay, &az);

	float q0 = m->q[0];
	float q1 = m->q[1];
	float q2 = m->q[2];
	float q3 = m->q[3];

	float ex = 0.0f;
	float ey = 0.0f;
	float ez = 0.0f;

	float hex = 0.0f;
	float hey = 0.0f;
	float hez = 0.0f;

	// predicted direction of gravity
	float vx = -2.0f*(q1*q3 - q0*q2);
	float vy = -2.0f*(q0*q1 + q2*q3);
	float vz = -(q0*q0 - q1*q1 - q2*q2 + q3*q3);

	if(accValid){
		ex = ay*vz - az*vy;
		ey = az*vx - ax*vz;
		ez = ax*vy - ay*vx;
	}

	/* compass: the heading error turned into a rotation about the earth vertical
	 * (v is "up" in body axes), so it only ever corrects the heading */
	float hb = 0.0f;
	float headingErr;
	if(magValid &&
		imu->gyro.x*imu->gyro.x + imu->gyro.y*imu->gyro.y + imu->gyro.z*imu->gyro.z < MAHONY_MAG_MAX_RATE*MAHONY_MAG_MAX_RATE &&
		Mahony_MagGeometry(m, mag, &headingErr, 0, 0)){
		if(m->magSnap && (!m->headingAligned || fabsf(headingErr) > m->magSnapRad)){
			/* start from the compass heading (or jump back to it on the ground,
			 * e.g. after a compass calibration) instead of crawling there */
			mahony_rotate_yaw(m, -headingErr);
			m->headingAligned = 1;
			q0 = m->q[0]; q1 = m->q[1]; q2 = m->q[2]; q3 = m->q[3];
			headingErr = 0.0f;
		}
		/* never aligned on the ground (it was turning the whole time): not used until it is,
		 * since the first alignment is a jump of any size */
		if(m->headingAligned){
			/* a reading that stands for magInterval seconds: all its correction in this one step */
			float k = (m->magInterval > 0.0f) ? m->magInterval / dt : 1.0f;
			float pe = mahony_clamp(headingErr, MAHONY_MAG_ERR_MAX);
			hex = pe * vx * k;
			hey = pe * vy * k;
			hez = pe * vz * k;
			hb	= m->magKi * k * mahony_clamp(headingErr, MAHONY_MAG_BIAS_ERR);
		}
	}

	/* the heading error also teaches the gyro bias (mostly z: the yaw drift) */
	m->bias[0] = mahony_clamp(m->bias[0] + (m->ki * ex + hb * vx) * dt, MAHONY_BIAS_LIMIT);
	m->bias[1] = mahony_clamp(m->bias[1] + (m->ki * ey + hb * vy) * dt, MAHONY_BIAS_LIMIT);
	m->bias[2] = mahony_clamp(m->bias[2] + (m->ki * ez + hb * vz) * dt, MAHONY_BIAS_LIMIT);

	m->rate[0] = imu->gyro.x + m->bias[0];
	m->rate[1] = imu->gyro.y + m->bias[1];
	m->rate[2] = imu->gyro.z + m->bias[2];

	float gx = m->rate[0] + m->kp * ex + m->magKp * hex;
	float gy = m->rate[1] + m->kp * ey + m->magKp * hey;
	float gz = m->rate[2] + m->kp * ez + m->magKp * hez;
	
	float dq0 = 0.5f * (-q1*gx - q2*gy - q3*gz);
	float dq1 = 0.5f * ( q0*gx + q2*gz - q3*gy);
	float dq2 = 0.5f * ( q0*gy - q1*gz + q3*gx);
	float dq3 = 0.5f * ( q0*gz + q1*gy - q2*gx);

	q0 += dq0 * dt;
	q1 += dq1 * dt;
	q2 += dq2 * dt;
	q3 += dq3 * dt;

	float norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm < 1e-6f) return;
	norm = 1.0f / norm;

	m->q[0] = q0 * norm;
	m->q[1] = q1 * norm;
	m->q[2] = q2 * norm;
	m->q[3] = q3 * norm;
}

void Mahony_GetQuaternion(const Mahony_t *m, float q[4]){
	q[0]=m->q[0];
	q[1]=m->q[1];
	q[2]=m->q[2];
	q[3]=m->q[3];
}

void Mahony_GetEuler(const Mahony_t *m, float *roll, float *pitch, float *yaw){
	float q0=m->q[0];
	float q1=m->q[1];
	float q2=m->q[2];
	float q3=m->q[3];

	*roll = atan2f(2.0f*(q0*q1+q2*q3), 1.0f-2.0f*(q1*q1+q2*q2));

	float s = 2.0f*(q0*q2-q3*q1);

	if(s>1.0f) s=1.0f;

	if(s<-1.0f) s=-1.0f;

	*pitch = asinf(s);

	*yaw = atan2f(2.0f*(q0*q3+q1*q2), 1.0f-2.0f*(q2*q2+q3*q3));
}

void Mahony_GetRate(const Mahony_t *m, float rate[3]){
	rate[0] = m->rate[0];
	rate[1] = m->rate[1];
	rate[2] = m->rate[2];
}

void Mahony_ResetHeading(Mahony_t *m){
	m->headingAligned = 0;
}