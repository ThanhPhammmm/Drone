#ifndef INC_TOPIC_ATTITUDE_TOPIC_H_
#define INC_TOPIC_ATTITUDE_TOPIC_H_

#include "topic.h"
#include <stdint.h>

typedef struct{
	float q0;
	float q1;
	float q2;
	float q3;

	float q[4];

	float roll;
	float pitch;
	float yaw;

	float rollRate;
	float pitchRate;
	float yawRate;

	float gyroBiasX;
	float gyroBiasY;
	float gyroBiasZ;

	float accelUp;
	float accelX;
	float accelY;

	float dt;

	/* compass, worked out even when it is not used (ATT_EST_USE_MAG 0) */
	float magHeading;		/* rad, -pi..pi, tilt compensated, 0 = north (declination added) */
	uint8_t magStatus;
	float magNorm;			/* gauss, field strength of the last reading; 0 = no reading */
	float magDip;			/* rad, its dip below the horizon */
	float magRefNorm;		/* gauss, learnt with the motors off; 0 = not learnt yet */
	float magRefDip;		/* rad */

	uint32_t timestamp_us;

} Attitude_Data_t;

#define ATT_MAG_FRESH	0x01	/* compass readings arrive (it is calibrated) */
#define ATT_MAG_OK		0x02	/* field strength and dip match the motors-off reference */
#define ATT_MAG_IN_USE	0x04	/* ATT_EST_USE_MAG on: the heading follows the compass, yaw 0 = north */
#define ATT_MAG_REF		0x08	/* the motors-off reference is learnt (~1 s of readings on the ground) */

extern Topic_t attitudeTopic;

BaseType_t AttitudeTopic_Init(void);
BaseType_t AttitudeTopic_Subscribe(TaskHandle_t task, const uint8_t id);
BaseType_t AttitudeTopic_Publish(const Attitude_Data_t *att);
BaseType_t AttitudeTopic_Copy(Attitude_Data_t *att);

#endif /* INC_TOPIC_ATTITUDE_TOPIC_H_ */
