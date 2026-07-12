#ifndef INC_APP_H_
#define INC_APP_H_

#define TASK_PRIO_IMU				5
#define TASK_PRIO_ESTIMATOR			4
#define TASK_PRIO_RATE				3
#define TASK_PRIO_ATTITUDE_CTRL		2
#define TASK_PRIO_MIXER				2

#define STACK_IMU					512
#define STACK_ESTIMATOR				512
#define STACK_ATTITUDE_CTRL			512
#define STACK_RATE					512
#define STACK_MIXER					256

void App_Init(void);

#endif /* INC_APP_H_ */
