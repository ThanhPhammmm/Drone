#ifndef INC_SENSORDRIVER_NEO_M8N_NEO_M8N_H_
#define INC_SENSORDRIVER_NEO_M8N_NEO_M8N_H_

#include "neo_m8n_data_types.h"
#include "neo_m8n_reg.h"
#include "usart.h"
#include <stdint.h>

#define NEO_M8N_UART            huart1
#define NEO_M8N_BAUD            115200U     /* the receiver is switched to this */
#define NEO_M8N_RATE_MS         200U        /* one solution every 200 ms (5 Hz) */
#define NEO_M8N_RX_BUF_LEN      512U        /* 44 ms of bytes at 115200 baud: poll faster than that */

typedef struct{
    NEO_M8N_Pvt_t pvt;              /* the last NAV-PVT */
    uint32_t      pvtCount;         /* NAV-PVT received so far */
    uint32_t      rxBytes;          /* bytes received so far */
    uint32_t      checksumErrors;
    uint32_t      baud;             /* MCU UART baud now */
} NEO_M8N_Handle_t;

extern NEO_M8N_Handle_t neoM8n;

NEO_M8N_Status_t NEO_M8N_StartRx(uint32_t baud);   /* MCU side: set the baud, (re)start the circular DMA */
NEO_M8N_Status_t NEO_M8N_SetPort(void);            /* receiver: NEO_M8N_BAUD, UBX in and out only */
NEO_M8N_Status_t NEO_M8N_Configure(void);          /* receiver: rate, NAV-PVT on, airborne model */
uint8_t          NEO_M8N_Poll(void);               /* parse what came in; 1 = a new NAV-PVT */

#endif /* INC_SENSORDRIVER_NEO_M8N_NEO_M8N_H_ */
