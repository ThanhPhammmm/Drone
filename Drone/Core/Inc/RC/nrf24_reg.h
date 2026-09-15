#ifndef INC_RC_NRF24_REG_H_
#define INC_RC_NRF24_REG_H_

#define NRF24_CMD_R_REGISTER        0x00
#define NRF24_CMD_W_REGISTER        0x20
#define NRF24_CMD_R_RX_PAYLOAD      0x61
#define NRF24_CMD_W_TX_PAYLOAD      0xA0
#define NRF24_CMD_FLUSH_TX          0xE1
#define NRF24_CMD_FLUSH_RX          0xE2
#define NRF24_CMD_NOP               0xFF

#define NRF24_REG_CONFIG            0x00
#define NRF24_REG_EN_AA             0x01
#define NRF24_REG_EN_RXADDR         0x02
#define NRF24_REG_SETUP_AW          0x03
#define NRF24_REG_SETUP_RETR        0x04
#define NRF24_REG_RF_CH             0x05
#define NRF24_REG_RF_SETUP          0x06
#define NRF24_REG_STATUS            0x07
#define NRF24_REG_OBSERVE_TX        0x08
#define NRF24_REG_RPD               0x09
#define NRF24_REG_RX_ADDR_P0        0x0A
#define NRF24_REG_TX_ADDR           0x10
#define NRF24_REG_RX_PW_P0          0x11
#define NRF24_REG_FIFO_STATUS       0x17
#define NRF24_REG_DYNPD             0x1C
#define NRF24_REG_FEATURE           0x1D

/* CONFIG */
#define NRF24_CFG_PRIM_RX           0x01
#define NRF24_CFG_PWR_UP            0x02
#define NRF24_CFG_CRCO              0x04    /* 2-byte CRC */
#define NRF24_CFG_EN_CRC            0x08
#define NRF24_CFG_MASK_MAX_RT       0x10
#define NRF24_CFG_MASK_TX_DS        0x20
#define NRF24_CFG_MASK_RX_DR        0x40

/* STATUS */
#define NRF24_STATUS_MAX_RT         0x10
#define NRF24_STATUS_TX_DS          0x20
#define NRF24_STATUS_RX_DR          0x40

/* FIFO_STATUS */
#define NRF24_FIFO_RX_EMPTY         0x01

/* RF_SETUP: 250 kbps for sensitivity/range, max PA for the LNA+PA module */
#define NRF24_RF_DR_250K            0x20
#define NRF24_RF_PWR_0DBM           0x06

#endif /* INC_RC_NRF24_REG_H_ */
