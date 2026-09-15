#ifndef INC_DRIVER_NRF24_TX_H_
#define INC_DRIVER_NRF24_TX_H_

#include "rc_protocol.h"
#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"

#define NRF24_SPI               hspi1
#define NRF24_CSN_PORT          GPIOB
#define NRF24_CSN_PIN           GPIO_PIN_0
#define NRF24_CE_PORT           GPIOA
#define NRF24_CE_PIN            GPIO_PIN_1

#define NRF24_CHANNEL           76      /* must match the drone's rc receiver */
#define NRF24_ADDR_WIDTH        5

typedef enum{
    NRF24_OK = 0,
    NRF24_ERROR,
    NRF24_TIMEOUT,
    NRF24_NOT_PRESENT,
} NRF24_Status_t;

NRF24_Status_t NRF24_TX_Init(void);
NRF24_Status_t NRF24_TX_Send(const RC_Packet_t *pkt, uint32_t timeoutMs);
void NRF24_SetTaskHandle(TaskHandle_t handle);
void NRF24_DMA_Callback(SPI_HandleTypeDef *hspi);

#endif /* INC_DRIVER_NRF24_TX_H_ */
