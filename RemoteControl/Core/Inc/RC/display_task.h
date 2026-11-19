#ifndef INC_RC_DISPLAY_TASK_H_
#define INC_RC_DISPLAY_TASK_H_

#include "stm32f4xx_hal.h"

/* Short press = next page, hold 1 s = back to page 1. */
#define DISP_BTN_PORT       GPIOB
#define DISP_BTN_PIN        GPIO_PIN_9

void DisplayTask(void *argument);

#endif /* INC_RC_DISPLAY_TASK_H_ */
