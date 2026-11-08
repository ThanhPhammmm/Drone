#include "attitude_topic.h"
#include "debug.h"

#define MAG_MAX_AGE_US		20000

#define MAHONY_KP_GROUND	1.0f
#define MAHONY_KP_FLIGHT	0.2f
#define MAHONY_KI			0.05f

static Mahony_t mahony;
AttitudeEstimator_Handle_t attitudeEstimator;

void AttitudeEstimator_SetTaskHandle(TaskHandle_t handle){
	attitudeEstimator.attitudeTask = handle;
}

void AttitudeEstimatorTask(void *argument){
	AttitudeEstimator_SetTaskHandle(xTaskGetCurrentTaskHandle());
	IMUTopic_Subscribe(attitudeEstimator.attitudeTask, ATTITUDE_ESTIMATOR_ID_TASK);

    BMI088_Data_t imu;
    Mahony_Init(&mahony, MAHONY_KP_GROUND, MAHONY_KI);
	uint8_t magWasUsed = 0;
	
    while(1){
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		if(IMUTopic_Copy(&imu) != pdPASS) continue;

        float ax = imu.accel.x;
        float ay = imu.accel.y;
        float az = imu.accel.z;

		uint8_t useMag = ATT_EST_USE_MAG;
		if(useMag && !magWasUsed) Mahony_ResetHeading(&mahony);
		magWasUsed = useMag;

		Mag_Data_t mag = {0};
		uint8_t magValid = 0;
		if(useMag && MagTopic_Copy(&mag, 0) == pdPASS && mag.timestamp_us != 0){
			int32_t age_us = Time_DiffUs(imu.timestamp_us, mag.timestamp_us);
		    if(age_us < MAG_MAX_AGE_US && age_us > -MAG_MAX_AGE_US){
		        magValid = 1;
		    }
		}
		float magVec[3] = { mag.x, mag.y, mag.z };

		mahony.kp = Arm_MotorsAllowed() ? MAHONY_KP_FLIGHT : MAHONY_KP_GROUND;
		Mahony_Update(&mahony, &imu, magVec, magValid);

		Attitude_Data_t *attitude = &attitudeEstimator.data;

		Mahony_GetQuaternion(&mahony, attitude->q);
		attitude->q0 = attitude->q[0];
		attitude->q1 = attitude->q[1];
		attitude->q2 = attitude->q[2];
		attitude->q3 = attitude->q[3];

		Mahony_GetEuler(&mahony, &attitude->roll, &attitude->pitch, &attitude->yaw);

	    float rate[3];
	    Mahony_GetRate(&mahony, rate);

		attitude->rollRate  = rate[0];
		attitude->pitchRate = rate[1];
		attitude->yawRate   = rate[2];

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
