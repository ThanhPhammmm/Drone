#ifndef INC_APP_H_
#define INC_APP_H_

#include "imu_topic.h"
#include "attitude_topic.h"
#include "rate_setpoint_topic.h"
#include "attitude_setpoint_topic.h"
#include "mag_topic.h"
#include "baro_topic.h"
#include "arm.h"
#include "altitude_topic.h"
#include "thrust_topic.h"
#include "rc_topic.h"
#include "altitude_setpoint_topic.h"
#include "actuator.h"

#define TASK_PRIO_IMU				6
#define TASK_PRIO_ATT_ESTIMATOR		5
#define TASK_PRIO_RATE				4
#define TASK_PRIO_ATT_CTRL			3

#define TASK_PRIO_ALT_ESTIMATOR		4
#define TASK_PRIO_ALT_CTRL			3
#define TASK_PRIO_MAGBARO			2
#define TASK_PRIO_RC				2

#define TASK_PRIO_MAG				1
#define TASK_PRIO_DEBUG				1

#define STACK_DEBUG					512
#define STACK_IMU					512
#define STACK_MAG					384
#define STACK_ATT_ESTIMATOR			512
#define STACK_ATTITUDE_CTRL			512
#define STACK_RATE					512
#define STACK_MIXER					256
#define STACK_MAGBARO				512
#define STACK_ALT_ESTIMATOR			512
#define STACK_ALT_CTRL			    512
#define STACK_RC					512

void App_Init(void);

#endif /* INC_APP_H_ */
