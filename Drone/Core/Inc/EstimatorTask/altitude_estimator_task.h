#ifndef INC_ESTIMATORTASK_ALTITUDE_ESTIMATOR_TASK_H_
#define INC_ESTIMATORTASK_ALTITUDE_ESTIMATOR_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "altitude_topic.h"

typedef struct{
    Altitude_Data_t data;
    TaskHandle_t altitudeTask;
} AltitudeEstimator_Handle_t;

void AltitudeEstimatorTask(void *argument);
void AltitudeEstimator_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_ESTIMATORTASK_ALTITUDE_ESTIMATOR_TASK_H_ */
