#ifndef INC_RC_RC_TASK_H_
#define INC_RC_RC_TASK_H_

#include "nrf24.h"
#include "rc_protocol.h"
#include "rc_topic.h"
#include "arm.h"
#include "attitude_setpoint_topic.h"
#include "altitude_setpoint_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stm32f4xx.h>
#include "debug.h"

void RCTask(void *argument);

#endif /* INC_RC_RC_TASK_H_ */
