#ifndef INC_RC_NRF24_H_
#define INC_RC_NRF24_H_

#include "rc_protocol.h"
#include "FreeRTOS.h"
#include "task.h"

#define NRF24_SPI               hspi2
#define NRF24_CSN_PORT          GPIOB
#define NRF24_CSN_PIN           GPIO_PIN_12
#define NRF24_CE_PORT           GPIOB
#define NRF24_CE_PIN            GPIO_PIN_0
#define NRF24_IRQ_PIN           GPIO_PIN_1      /* EXTI1, falling edge */

#define NRF24_CHANNEL           76
#define NRF24_ADDR_WIDTH        5

typedef enum{
    NRF24_OK = 0,
    NRF24_ERROR,
    NRF24_EMPTY,
    NRF24_NOT_PRESENT,
} NRF24_Status_t;

NRF24_Status_t NRF24_Init(void);
NRF24_Status_t NRF24_ReadPacket(RC_Packet_t *pkt);
void NRF24_SetTaskHandle(TaskHandle_t handle);
void NRF24_IRQ_Handler(void);

#endif /* INC_RC_NRF24_H_ */
