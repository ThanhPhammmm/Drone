#include "nrf24_tx.h"
#include "nrf24_reg.h"
#include <string.h>
#include <semphr.h>

#define NRF24_SPI_TIMEOUT_MS    10
static TaskHandle_t rcTaskHandle = NULL;
SemaphoreHandle_t nrf24DmaDone = NULL;

/* Must be identical to the address programmed into RX_ADDR_P0 on the drone */

static const uint8_t nrf24Address[NRF24_ADDR_WIDTH] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

void NRF24_SetTaskHandle(TaskHandle_t handle){
    rcTaskHandle = handle;
}

static inline void NRF24_CSN(GPIO_PinState s){
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, s);
}
static inline void NRF24_CE(GPIO_PinState s){
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, s);
}

void NRF24_DMA_Callback(SPI_HandleTypeDef *hspi){
    if(nrf24DmaDone == NULL) return;

    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(nrf24DmaDone, &hpw);
    portYIELD_FROM_ISR(hpw);
}


static uint8_t NRF24_Transfer(uint8_t *tx, uint8_t *rx, uint16_t len){
    NRF24_CSN(GPIO_PIN_RESET);
    if(HAL_SPI_TransmitReceive_DMA(&NRF24_SPI, tx, rx, len) != HAL_OK){
        NRF24_CSN(GPIO_PIN_SET);
        return 0;
    }
//    if(HAL_SPI_TransmitReceive(&NRF24_SPI, tx, rx, len, NRF24_SPI_TIMEOUT_MS) != HAL_OK){
//        NRF24_CSN(GPIO_PIN_SET);
//        return 0;
//    }
    uint8_t ok = (xSemaphoreTake(nrf24DmaDone, pdMS_TO_TICKS(NRF24_SPI_TIMEOUT_MS)) == pdTRUE);
    if(!ok){
        HAL_SPI_Abort(&NRF24_SPI);
        NRF24_CSN(GPIO_PIN_SET);
        return 0;
    }
    uint32_t guard = 10000;
    while((NRF24_SPI.Instance->SR & SPI_SR_BSY) && --guard);
    NRF24_CSN(GPIO_PIN_SET);
    return ok;
    //return 1;
}

static uint8_t NRF24_ReadReg(uint8_t reg){
	uint8_t tx[2] = { NRF24_CMD_R_REGISTER | reg, NRF24_CMD_NOP };
	uint8_t rx[2] = { 0 };
    NRF24_Transfer(tx, rx, 2);
    return rx[1];
}

static void NRF24_WriteReg(uint8_t reg, uint8_t val){
	uint8_t tx[2] = { NRF24_CMD_W_REGISTER | reg, val };
	uint8_t rx[2];
    NRF24_Transfer(tx, rx, 2);
}

static void NRF24_WriteRegMulti(uint8_t reg, const uint8_t *buf, uint8_t len){
    uint8_t tx[1 + NRF24_ADDR_WIDTH];
    uint8_t rx[1 + NRF24_ADDR_WIDTH];

    tx[0] = NRF24_CMD_W_REGISTER | reg;
    memcpy(&tx[1], buf, len);
    NRF24_Transfer(tx, rx, len + 1);
}

static void NRF24_Command(uint8_t cmd){
    uint8_t tx = cmd, rx;
    NRF24_Transfer(&tx, &rx, 1);
}

NRF24_Status_t NRF24_TX_Init(void){
    if(nrf24DmaDone == NULL){
        nrf24DmaDone = xSemaphoreCreateBinary();
    }
    NRF24_CE(GPIO_PIN_RESET);
    NRF24_CSN(GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(150));       /* power-on settle, same as RX side */

    /* Presence check, same trick as the drone side: RF_CH is freely
     * writable, so a read-back mismatch means the module is absent/miswired. */
    uint8_t fifo = NRF24_ReadReg(NRF24_REG_FIFO_STATUS);
    uint8_t status = NRF24_ReadReg(NRF24_REG_STATUS);
    NRF24_WriteReg(NRF24_REG_RF_CH, NRF24_CHANNEL);
    NRF24_WriteReg(NRF24_REG_RF_CH, NRF24_CHANNEL);
    fifo = NRF24_ReadReg(NRF24_REG_FIFO_STATUS);
    status = NRF24_ReadReg(NRF24_REG_STATUS);
    if(NRF24_ReadReg(NRF24_REG_RF_CH) != NRF24_CHANNEL){
        return NRF24_NOT_PRESENT;
    }
    fifo = NRF24_ReadReg(NRF24_REG_FIFO_STATUS);
    status = NRF24_ReadReg(NRF24_REG_STATUS);
    NRF24_WriteReg(NRF24_REG_EN_AA,      0x01);   /* auto-ack on pipe 0 */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR,  0x01);   /* pipe 0 used to receive the ACK */
    NRF24_WriteReg(NRF24_REG_SETUP_AW,   0x03);   /* 5-byte address */
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x2F);   /* 750us delay, 15 retries */
    NRF24_WriteReg(NRF24_REG_RF_SETUP,   NRF24_RF_DR_250K | NRF24_RF_PWR_0DBM);
    NRF24_WriteReg(NRF24_REG_DYNPD,      0x00);   /* static payload width */
    NRF24_WriteReg(NRF24_REG_FEATURE,    0x00);
    NRF24_WriteReg(NRF24_REG_RX_PW_P0,   sizeof(RC_Packet_t));

    /* TX_ADDR and RX_ADDR_P0 must match for the ACK to come back on pipe 0. */
    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24Address, NRF24_ADDR_WIDTH);
    NRF24_WriteRegMulti(NRF24_REG_TX_ADDR,    nrf24Address, NRF24_ADDR_WIDTH);

    NRF24_WriteReg(NRF24_REG_CONFIG, NRF24_CFG_EN_CRC | NRF24_CFG_CRCO | NRF24_CFG_PWR_UP);
    vTaskDelay(pdMS_TO_TICKS(2));       /* power-up -> standby */

    NRF24_Command(NRF24_CMD_FLUSH_TX);
    NRF24_WriteReg(NRF24_REG_STATUS,
        NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    return NRF24_OK;
}

NRF24_Status_t NRF24_TX_Send(const RC_Packet_t *pkt, uint32_t timeoutMs){
    /* Clear out anything left over from a previous failed send. */
    NRF24_Command(NRF24_CMD_FLUSH_TX);
    NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    uint8_t tx[1 + sizeof(RC_Packet_t)] = { NRF24_CMD_W_TX_PAYLOAD };
    uint8_t rx[1 + sizeof(RC_Packet_t)];
    memcpy(&tx[1], pkt, sizeof(RC_Packet_t));

    if(!NRF24_Transfer(tx, rx, sizeof(tx))) return NRF24_ERROR;
    NRF24_CE(GPIO_PIN_SET);
    vTaskDelay(pdMS_TO_TICKS(1));
    NRF24_CE(GPIO_PIN_RESET);
    TickType_t start = xTaskGetTickCount();
    while((xTaskGetTickCount() - start) < pdMS_TO_TICKS(timeoutMs)){
        uint8_t status = NRF24_ReadReg(NRF24_REG_STATUS);

        if(status & NRF24_STATUS_TX_DS){
            NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_TX_DS);
            return NRF24_OK;
        }
        if(status & NRF24_STATUS_MAX_RT){
            NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_MAX_RT);
            NRF24_Command(NRF24_CMD_FLUSH_TX);
            return NRF24_ERROR;    /* drone out of range / powered off */
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    NRF24_Command(NRF24_CMD_FLUSH_TX);
    return NRF24_TIMEOUT;
}
