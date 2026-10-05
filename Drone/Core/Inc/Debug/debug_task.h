#ifndef INC_DEBUG_DEBUG_TASK_H_
#define INC_DEBUG_DEBUG_TASK_H_

/* 1: debug task running.
 * 0: flight build, no task, no UART traffic. */

#define DEBUG_PRINT			1
#define DEBUG_PERIOD_MS		1000U

void DebugTask(void *argument);

#endif /* INC_DEBUG_DEBUG_TASK_H_ */
