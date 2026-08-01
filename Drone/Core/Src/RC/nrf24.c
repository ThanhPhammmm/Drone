#include "nrf24.h"
#include "nrf24_reg.h"
#include "spi.h"
#include <string.h>

#define NRF24_SPI_TIMEOUT_MS    10

static TaskHandle_t rcTaskHandle = NULL;

static const uint8_t nrf24Address[NRF24_ADDR_WIDTH] = { 0xE7, 0xE7, 0xE7, 0xE7, 0xE7 };

static inline void NRF24_CSN(GPIO_PinState s){
    HAL_GPIO_WritePin(NRF24_CSN_PORT, NRF24_CSN_PIN, s);
}
static inline void NRF24_CE(GPIO_PinState s){
    HAL_GPIO_WritePin(NRF24_CE_PORT, NRF24_CE_PIN, s);
}

void NRF24_SetTaskHandle(TaskHandle_t handle){
    rcTaskHandle = handle;
}

void NRF24_IRQ_Handler(void){
    if(rcTaskHandle == NULL) return;

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(rcTaskHandle, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static uint8_t NRF24_Transfer(uint8_t *tx, uint8_t *rx, uint16_t len){
    NRF24_CSN(GPIO_PIN_RESET);
    HAL_StatusTypeDef st = HAL_SPI_TransmitReceive(&NRF24_SPI, tx, rx, len, NRF24_SPI_TIMEOUT_MS);
    NRF24_CSN(GPIO_PIN_SET);
    return (st == HAL_OK);
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

NRF24_Status_t NRF24_Init(void){
    NRF24_CE(GPIO_PIN_RESET);
    NRF24_CSN(GPIO_PIN_SET);
    HAL_Delay(5);                       /* power-on settle */

    /* Presence check: RF_CH is freely writable, so a read-back mismatch means
     * the module is absent or miswired rather than merely misconfigured. */
    NRF24_WriteReg(NRF24_REG_RF_CH, NRF24_CHANNEL);
    if(NRF24_ReadReg(NRF24_REG_RF_CH) != NRF24_CHANNEL){
        return NRF24_NOT_PRESENT;
    }

    NRF24_WriteReg(NRF24_REG_EN_AA,      0x01);   /* auto-ack on pipe 0 */
    NRF24_WriteReg(NRF24_REG_EN_RXADDR,  0x01);
    NRF24_WriteReg(NRF24_REG_SETUP_AW,   0x03);   /* 5-byte address */
    NRF24_WriteReg(NRF24_REG_SETUP_RETR, 0x00);   /* RX side: no retransmit */
    NRF24_WriteReg(NRF24_REG_RF_SETUP,   NRF24_RF_DR_250K | NRF24_RF_PWR_0DBM);
    NRF24_WriteReg(NRF24_REG_DYNPD,      0x00);   /* static payload width */
    NRF24_WriteReg(NRF24_REG_FEATURE,    0x00);
    NRF24_WriteReg(NRF24_REG_RX_PW_P0,   sizeof(RC_Packet_t));

    NRF24_WriteRegMulti(NRF24_REG_RX_ADDR_P0, nrf24Address, NRF24_ADDR_WIDTH);
    NRF24_WriteRegMulti(NRF24_REG_TX_ADDR,    nrf24Address, NRF24_ADDR_WIDTH);

    /* RX mode, 2-byte CRC, only RX_DR allowed to pull IRQ low */
    NRF24_WriteReg(NRF24_REG_CONFIG,
        NRF24_CFG_MASK_TX_DS | NRF24_CFG_MASK_MAX_RT |
        NRF24_CFG_EN_CRC | NRF24_CFG_CRCO |
        NRF24_CFG_PWR_UP | NRF24_CFG_PRIM_RX);

    HAL_Delay(2);                       /* power-up -> standby */

    NRF24_Command(NRF24_CMD_FLUSH_RX);
    NRF24_WriteReg(NRF24_REG_STATUS,
        NRF24_STATUS_RX_DR | NRF24_STATUS_TX_DS | NRF24_STATUS_MAX_RT);

    NRF24_CE(GPIO_PIN_SET);             /* start listening, stays high */

    return NRF24_OK;
}

NRF24_Status_t NRF24_ReadPacket(RC_Packet_t *pkt){
    if(NRF24_ReadReg(NRF24_REG_FIFO_STATUS) & NRF24_FIFO_RX_EMPTY){
        return NRF24_EMPTY;
    }

    uint8_t tx[1 + sizeof(RC_Packet_t)] = { NRF24_CMD_R_RX_PAYLOAD };
    uint8_t rx[1 + sizeof(RC_Packet_t)];

    if(!NRF24_Transfer(tx, rx, sizeof(RC_Packet_t) + 1)) return NRF24_ERROR;

    memcpy(pkt, &rx[1], sizeof(RC_Packet_t));

    NRF24_WriteReg(NRF24_REG_STATUS, NRF24_STATUS_RX_DR);

    return NRF24_OK;
}
