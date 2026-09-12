#ifndef INC_SENSORTASK_MAGBARO_TASK_H_
#define INC_SENSORTASK_MAGBARO_TASK_H_

#include "qmc5883.h"
#include "bmp388.h"
#include "mag_topic.h"
#include "baro_topic.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stm32f4xx.h>

void MagBaroTask(void *argument);

#endif /* INC_SENSORTASK_MAGBARO_TASK_H_ */
