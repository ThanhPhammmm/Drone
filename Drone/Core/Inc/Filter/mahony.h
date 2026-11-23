#ifndef INC_FILTER_MAHONY_MAHONY_H_
#define INC_FILTER_MAHONY_MAHONY_H_

#include "bmi088_data_types.h"

typedef struct{
	float kp;
	float ki;

	/* compass (only used when Mahony_Update gets magValid = 1) */
	float magKp;			/* rad/s of heading correction per rad of heading error */
	float magKi;			/* how fast the heading error teaches the gyro bias (yaw drift) */
	float declination;		/* rad, magnetic north -> true north, east + */
	uint8_t magSnap;		/* 1 (ground only): the first reading aligns the heading to the
							 * compass, and an error over magSnapRad jumps it there at once.
							 * 0 (flight): no jumps; until aligned the compass is not used */
	float magSnapRad;
	float magInterval;		/* s: the time one compass reading stands for. Its whole correction
							 * is applied on the step it arrives (magValid = 1 on that step only),
							 * so an aging reading is never compared with a newer heading.
							 * 0 = apply magKp/magKi on every step magValid is 1 */

	/* quaternion */
	float q[4];

	/* estimated gyro bias */
	float bias[3];

	/* bias-corrected body rates (rad/s) */
	float rate[3];

	uint8_t headingAligned;	/* yaw snapped to the magnetometer once */

} Mahony_t;

void Mahony_Init(Mahony_t *mahony, float kp, float ki);
void Mahony_Update(Mahony_t *mahony, const BMI088_Data_t *imu, const float mag[3], uint8_t magValid);
void Mahony_GetQuaternion(const Mahony_t *mahony, float q[4]);
void Mahony_GetEuler(const Mahony_t *mahony, float *roll, float *pitch, float *yaw);
void Mahony_GetRate(const Mahony_t *mahony, float rate[3]);
void Mahony_ResetHeading(Mahony_t *mahony);

/* The compass reading seen through the current attitude (mag: calibrated field, body
 * axes FRD, any unit). headingErr: how far the estimated heading is turned clockwise
 * past the compass heading (rad, -pi..pi): the compass heading is yaw - headingErr.
 * dip: angle of the field below the horizon (rad). norm: field strength, mag's unit.
 * 0 = no usable reading (zero field, or a field straight up/down). */
uint8_t Mahony_MagGeometry(const Mahony_t *mahony, const float mag[3], float *headingErr, float *dip, float *norm);

#endif /* INC_FILTER_MAHONY_MAHONY_H_ */
