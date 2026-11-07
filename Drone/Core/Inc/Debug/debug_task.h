#ifndef INC_DEBUG_DEBUG_TASK_H_
#define INC_DEBUG_DEBUG_TASK_H_

/* 1: debug task running.
 * 0: flight build, no task, no UART traffic.
 * USART1 is the GPS now: with 1, the debug prints would go to the GPS. 
 * Use telemetry for debug */

#define DEBUG_PRINT			0
#define DEBUG_PERIOD_MS		1000U

void DebugTask(void *argument);

#endif /* INC_DEBUG_DEBUG_TASK_H_ */
