#ifndef INC_CONTROLLERTASK_ALTITUDE_CONTROLLER_TASK_H_
#define INC_CONTROLLERTASK_ALTITUDE_CONTROLLER_TASK_H_

#include "FreeRTOS.h"
#include "task.h"
#include "thrust_topic.h"

typedef struct{
    Thrust_Data_t data;
    TaskHandle_t  controllerTask;

    float holdTarget;       /* m, commanded altitude */
    float vzSetpoint;       /* m/s */
} AltitudeController_Handle_t;

extern AltitudeController_Handle_t altitudeController;

void AltitudeControllerTask(void *argument);
void AltitudeController_SetTaskHandle(TaskHandle_t handle);

#endif /* INC_CONTROLLERTASK_ALTITUDE_CONTROLLER_TASK_H_ */
