#ifndef INC_RC_RC_TX_TASK_H_
#define INC_RC_RC_TX_TASK_H_

#include "main.h"	/* CubeMX pin labels, if given */
#include "rc_protocol.h"
#include <stdint.h>
#include "telemetry.h"

#if defined(CAL_SW_Pin) && defined(CAL_SW_GPIO_Port)
#define CAL_SW_PORT		CAL_SW_GPIO_Port
#define CAL_SW_PIN		CAL_SW_Pin
#else
#define CAL_SW_PORT		GPIOB
#define CAL_SW_PIN		GPIO_PIN_15
#endif
#define CAL_SW_DEBOUNCE	3	/* packets (20 ms each) the same before a change counts */

typedef struct{
	RC_Packet_t lastSent;
	uint8_t linkOk;	/* drone ACKed the last packet */
	uint8_t radioOk;	/* nRF24 present and initialized */
} RCTx_Status_t;


void RCTxTask(void *argument);
uint8_t RCTx_GetStatus(RCTx_Status_t *out);
void RCTx_CalStart(uint8_t item);	/* TFT page 6: ask the drone to calibrate RC_CalItem_t item */
void RCTx_CalCancel(void);	/* stop the calibration that runs */

#endif /* INC_RC_RC_TX_TASK_H_ */
