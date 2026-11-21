#ifndef INC_RC_DISPLAY_TASK_H_
#define INC_RC_DISPLAY_TASK_H_

#include "main.h"

/* Three buttons, each from its pin to GND. Set up in CubeMX: GPIO_Input, Pull-up.
 *   PREV  PB9   previous page;  page 6: arrow up
 *   NEXT  PA8   next page;      page 6: arrow down
 *   OK    PA15  back to page 1; page 6: start / cancel the running one / EXIT
 * With the CubeMX labels BTN_PREV, BTN_NEXT, BTN_OK the pins come from main.h. */
#if defined(BTN_PREV_Pin) && defined(BTN_PREV_GPIO_Port)
#define BTN_PREV_PORT		BTN_PREV_GPIO_Port
#define BTN_PREV_PIN		BTN_PREV_Pin
#else
#define BTN_PREV_PORT		GPIOB
#define BTN_PREV_PIN		GPIO_PIN_9
#endif
#if defined(BTN_NEXT_Pin) && defined(BTN_NEXT_GPIO_Port)
#define BTN_NEXT_PORT		BTN_NEXT_GPIO_Port
#define BTN_NEXT_PIN		BTN_NEXT_Pin
#else
#define BTN_NEXT_PORT		GPIOA
#define BTN_NEXT_PIN		GPIO_PIN_8
#endif
#if defined(BTN_OK_Pin) && defined(BTN_OK_GPIO_Port)
#define BTN_OK_PORT			BTN_OK_GPIO_Port
#define BTN_OK_PIN			BTN_OK_Pin
#else
#define BTN_OK_PORT			GPIOA
#define BTN_OK_PIN			GPIO_PIN_15
#endif

void DisplayTask(void *argument);

#endif /* INC_RC_DISPLAY_TASK_H_ */
