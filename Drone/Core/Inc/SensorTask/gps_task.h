#ifndef INC_SENSORTASK_GPS_TASK_H_
#define INC_SENSORTASK_GPS_TASK_H_

#include "neo_m8n.h"
#include "gps_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timebase.h"

void    GpsTask(void *argument);
uint8_t Gps_Found(void);        /* the receiver answers with NAV-PVT */

#endif /* INC_SENSORTASK_GPS_TASK_H_ */
