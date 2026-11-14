#include "neo_m8n.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define UBX_LENGTH_SANE         1024U       /* longer: a broken length, look for the next frame */
#define UBX_TX_PAYLOAD_MAX      UBX_CFG_NAV5_LEN    /* longest message sent */
#define UBX_TX_FRAME_MAX        (UBX_HEADER_LEN + UBX_TX_PAYLOAD_MAX + UBX_CHECKSUM_LEN)

NEO_M8N_Handle_t neoM8n;

static volatile uint8_t rxBuf[NEO_M8N_RX_BUF_LEN];     /* filled by the DMA, round and round */
static uint8_t txFrame[UBX_TX_FRAME_MAX];              /* read by the TX DMA: must outlive UBX_Send */
static uint16_t rxRead;                                 /* next byte to parse */
static UBX_Parser_t ubx;

static uint16_t UBX_U2(const uint8_t *p){
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t UBX_U4(const uint8_t *p){
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t UBX_I4(const uint8_t *p){
    return (int32_t)UBX_U4(p);
}

static void UBX_PutU2(uint8_t *p, uint16_t v){
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static void UBX_PutU4(uint8_t *p, uint32_t v){
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static void UBX_AddToChecksum(uint8_t b){
    ubx.ckA = (uint8_t)(ubx.ckA + b);
    ubx.ckB = (uint8_t)(ubx.ckB + ubx.ckA);
}

/* NAV-PVT payload -> neoM8n.pvt */
static void UBX_DecodePvt(void){
    const uint8_t *p = ubx.payload;
    NEO_M8N_Pvt_t *v = &neoM8n.pvt;

    v->iTOW    = UBX_U4(p + UBX_PVT_ITOW);
    v->fixType = p[UBX_PVT_FIX_TYPE];
    v->fixOk   = p[UBX_PVT_FLAGS] & UBX_PVT_FLAG_FIX_OK;
    v->numSV   = p[UBX_PVT_NUM_SV];
    v->lon     = UBX_I4(p + UBX_PVT_LON);
    v->lat     = UBX_I4(p + UBX_PVT_LAT);
    v->hMSL    = UBX_I4(p + UBX_PVT_HMSL);
    v->hAcc    = UBX_U4(p + UBX_PVT_HACC);
    v->vAcc    = UBX_U4(p + UBX_PVT_VACC);
    v->velN    = UBX_I4(p + UBX_PVT_VELN);
    v->velE    = UBX_I4(p + UBX_PVT_VELE);
    v->velD    = UBX_I4(p + UBX_PVT_VELD);
    v->sAcc    = UBX_U4(p + UBX_PVT_SACC);
    v->pDOP    = UBX_U2(p + UBX_PVT_PDOP);
    neoM8n.pvtCount++;
}

/* one byte in; 1 = it completed a good NAV-PVT */
static uint8_t UBX_Parse(uint8_t b){
    switch(ubx.state){
    case UBX_WAIT_SYNC_1:
        if(b == UBX_SYNC_CHAR_1) ubx.state = UBX_WAIT_SYNC_2;
        break;

    case UBX_WAIT_SYNC_2:
        if(b == UBX_SYNC_CHAR_2){
            ubx.ckA   = 0;
            ubx.ckB   = 0;
            ubx.state = UBX_WAIT_CLASS;
        }
        else{
            ubx.state = (b == UBX_SYNC_CHAR_1) ? UBX_WAIT_SYNC_2 : UBX_WAIT_SYNC_1;
        }
        break;

    case UBX_WAIT_CLASS:
        ubx.cls = b;
        UBX_AddToChecksum(b);
        ubx.state = UBX_WAIT_ID;
        break;

    case UBX_WAIT_ID:
        ubx.id = b;
        UBX_AddToChecksum(b);
        ubx.state = UBX_WAIT_LEN_1;
        break;

    case UBX_WAIT_LEN_1:
        ubx.len = b;
        UBX_AddToChecksum(b);
        ubx.state = UBX_WAIT_LEN_2;
        break;

    case UBX_WAIT_LEN_2:
        ubx.len  |= (uint16_t)b << 8;
        ubx.count = 0;
        UBX_AddToChecksum(b);
        if(ubx.len > UBX_LENGTH_SANE) ubx.state = UBX_WAIT_SYNC_1;
        else                          ubx.state = (ubx.len == 0) ? UBX_WAIT_CK_A : UBX_WAIT_PAYLOAD;
        break;

    case UBX_WAIT_PAYLOAD:
        if(ubx.count < UBX_PAYLOAD_MAX) ubx.payload[ubx.count] = b;     /* longer ones: only checked */
        ubx.count++;
        UBX_AddToChecksum(b);
        if(ubx.count >= ubx.len) ubx.state = UBX_WAIT_CK_A;
        break;

    case UBX_WAIT_CK_A:
        if(b == ubx.ckA){
            ubx.state = UBX_WAIT_CK_B;
        }
        else{
            neoM8n.checksumErrors++;
            ubx.state = (b == UBX_SYNC_CHAR_1) ? UBX_WAIT_SYNC_2 : UBX_WAIT_SYNC_1;
        }
        break;

    case UBX_WAIT_CK_B:
        ubx.state = UBX_WAIT_SYNC_1;
        if(b != ubx.ckB){
            neoM8n.checksumErrors++;
            if(b == UBX_SYNC_CHAR_1) ubx.state = UBX_WAIT_SYNC_2;
            break;
        }
        if(ubx.cls == UBX_CLASS_NAV && ubx.id == UBX_NAV_PVT && ubx.len == UBX_NAV_PVT_LEN){
            UBX_DecodePvt();
            return 1;
        }
        break;

    default:
        ubx.state = UBX_WAIT_SYNC_1;
        break;
    }
    return 0;
}

static NEO_M8N_Status_t UBX_WaitTxDone(void){
    TickType_t start = xTaskGetTickCount();
    while(NEO_M8N_UART.gState != HAL_UART_STATE_READY){
        if((TickType_t)(xTaskGetTickCount() - start) > pdMS_TO_TICKS(NEO_M8N_TX_TIMEOUT_MS)){
            (void)HAL_UART_AbortTransmit(&NEO_M8N_UART);    /* stuck: USART1 global interrupt off? */
            return NEO_M8N_ERROR;
        }
        vTaskDelay(1);
    }
    return NEO_M8N_OK;
}

static NEO_M8N_Status_t UBX_Send(uint8_t cls, uint8_t id, const uint8_t *payload, uint16_t len){
    if(len > UBX_TX_PAYLOAD_MAX) return NEO_M8N_ERROR;
    if(UBX_WaitTxDone() != NEO_M8N_OK) return NEO_M8N_ERROR;

    txFrame[0] = UBX_SYNC_CHAR_1;
    txFrame[1] = UBX_SYNC_CHAR_2;
    txFrame[2] = cls;
    txFrame[3] = id;
    UBX_PutU2(&txFrame[4], len);
    if(len) memcpy(&txFrame[6], payload, len);

    uint8_t a = 0, b = 0;
    for(uint16_t i = 2; i < 6U + len; i++){
        a = (uint8_t)(a + txFrame[i]);
        b = (uint8_t)(b + a);
    }
    txFrame[6U + len] = a;
    txFrame[7U + len] = b;

    if(HAL_UART_Transmit_DMA(&NEO_M8N_UART, txFrame, (uint16_t)(UBX_HEADER_LEN + len + UBX_CHECKSUM_LEN)) != HAL_OK){
        return NEO_M8N_ERROR;
    }
    return UBX_WaitTxDone();
}

NEO_M8N_Status_t NEO_M8N_StartRx(uint32_t baud){
    (void)HAL_UART_DMAStop(&NEO_M8N_UART);
    NEO_M8N_UART.Init.BaudRate = baud;
    if(HAL_UART_Init(&NEO_M8N_UART) != HAL_OK) return NEO_M8N_ERROR;

    rxRead    = 0;
    ubx.state = UBX_WAIT_SYNC_1;
    if(HAL_UART_Receive_DMA(&NEO_M8N_UART, (uint8_t *)rxBuf, NEO_M8N_RX_BUF_LEN) != HAL_OK){
        return NEO_M8N_ERROR;
    }
    /* A noise, framing or overrun error would make the HAL stop the RX DMA
     * from the USART interrupt. Keep it running instead: a broken frame
     * fails its checksum and the next one is fine. */
    __HAL_UART_DISABLE_IT(&NEO_M8N_UART, UART_IT_ERR);
    __HAL_UART_DISABLE_IT(&NEO_M8N_UART, UART_IT_PE);

    neoM8n.baud = baud;
    return NEO_M8N_OK;
}

/* CFG-PRT: UART1 at NEO_M8N_BAUD, 8N1, UBX in and out (NMEA off).
 * Sent at the baud the receiver runs at now; it answers at the new one. */
NEO_M8N_Status_t NEO_M8N_SetPort(void){
    uint8_t p[UBX_CFG_PRT_LEN] = {0};
    p[0] = UBX_PRT_PORT_UART1;
    UBX_PutU4(&p[4],  UBX_PRT_MODE_8N1);
    UBX_PutU4(&p[8],  NEO_M8N_BAUD);
    UBX_PutU2(&p[12], UBX_PRT_PROTO_UBX);      /* in */
    UBX_PutU2(&p[14], UBX_PRT_PROTO_UBX);      /* out */
    return UBX_Send(UBX_CLASS_CFG, UBX_CFG_PRT, p, sizeof(p));
}

/* CFG-RATE: one solution every NEO_M8N_RATE_MS. CFG-MSG: NAV-PVT with every
 * solution on this port. CFG-NAV5: the airborne model (the default one
 * expects a car and smooths a drone's motion away). Not saved in the
 * receiver: sent again at every start. */
NEO_M8N_Status_t NEO_M8N_Configure(void){
    NEO_M8N_Status_t st = NEO_M8N_OK;

    uint8_t rate[UBX_CFG_RATE_LEN];
    UBX_PutU2(&rate[0], NEO_M8N_RATE_MS);
    UBX_PutU2(&rate[2], 1U);
    UBX_PutU2(&rate[4], UBX_RATE_TIME_REF_GPS);
    if(UBX_Send(UBX_CLASS_CFG, UBX_CFG_RATE, rate, sizeof(rate)) != NEO_M8N_OK) st = NEO_M8N_ERROR;
    vTaskDelay(pdMS_TO_TICKS(NEO_M8N_CFG_GAP_MS));

    const uint8_t msg[UBX_CFG_MSG_LEN] = { UBX_CLASS_NAV, UBX_NAV_PVT, 1U };
    if(UBX_Send(UBX_CLASS_CFG, UBX_CFG_MSG, msg, sizeof(msg)) != NEO_M8N_OK) st = NEO_M8N_ERROR;
    vTaskDelay(pdMS_TO_TICKS(NEO_M8N_CFG_GAP_MS));

    uint8_t nav5[UBX_CFG_NAV5_LEN] = {0};
    UBX_PutU2(&nav5[0], UBX_NAV5_MASK_DYN);
    nav5[2] = UBX_NAV5_DYN_AIRBORNE_1G;
    if(UBX_Send(UBX_CLASS_CFG, UBX_CFG_NAV5, nav5, sizeof(nav5)) != NEO_M8N_OK) st = NEO_M8N_ERROR;
    vTaskDelay(pdMS_TO_TICKS(NEO_M8N_CFG_GAP_MS));

    return st;
}

uint8_t NEO_M8N_Poll(void){
    uint16_t write = (uint16_t)(NEO_M8N_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(NEO_M8N_UART.hdmarx));
    if(write >= NEO_M8N_RX_BUF_LEN) write = 0;

    uint8_t gotPvt = 0;
    while(rxRead != write){
        if(UBX_Parse(rxBuf[rxRead])) gotPvt = 1;
        rxRead = (uint16_t)((rxRead + 1U) % NEO_M8N_RX_BUF_LEN);
        neoM8n.rxBytes++;
    }
    return gotPvt;
}
