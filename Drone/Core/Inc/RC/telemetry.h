#ifndef INC_RC_TELEMETRY_H_
#define INC_RC_TELEMETRY_H_

#include <stdint.h>
#include "rc_protocol.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <math.h>
#include "arm.h"
#include "flight_mode.h"
#include "bmi088.h"
#include "magbaro_task.h"
#include "imu_topic.h"
#include "attitude_topic.h"
#include "attitude_setpoint_topic.h"
#include "rate_setpoint_topic.h"
#include "altitude_setpoint_topic.h"
#include "altitude_topic.h"
#include "baro_topic.h"
#include "thrust_topic.h"
#include "actuator.h"
#include "rate_controller_task.h"

uint8_t Telemetry_Build(uint8_t *buf, uint8_t rcSeq, uint16_t rcLost);

#endif /* INC_RC_TELEMETRY_H_ */
