#ifndef INC_SENSORTASK_RANGEFLOW_TASK_H_
#define INC_SENSORTASK_RANGEFLOW_TASK_H_

#include "pmw3901.h"
#include "vl53l1x.h"
#include "flow_topic.h"
#include "range_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timebase.h"
#include "i2c.h"

void RangeFlowTask(void *argument);
uint8_t RangeFlow_RangeReady(void);    /* VL53L1X initialized OK */
uint8_t RangeFlow_FlowReady(void);     /* PMW3901 initialized OK */

#endif /* INC_SENSORTASK_RANGEFLOW_TASK_H_ */