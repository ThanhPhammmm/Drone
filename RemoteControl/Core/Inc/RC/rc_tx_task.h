#ifndef INC_RC_RC_TX_TASK_H_
#define INC_RC_RC_TX_TASK_H_

#include "rc_protocol.h"
#include <stdint.h>

typedef struct{
    RC_Packet_t lastSent;
    uint8_t     linkOk;      /* drone ACKed the last packet */
    uint8_t     radioOk;     /* nRF24 present and initialized */
} RCTx_Status_t;


void RCTxTask(void *argument);
uint8_t RCTx_GetStatus(RCTx_Status_t *out);

#endif /* INC_RC_RC_TX_TASK_H_ */
