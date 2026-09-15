#include "rc_tx_task.h"
#include "nrf24_tx.h"
#include "joystick.h"
#include "mode_switch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define TX_PERIOD_MS           20    /* 50Hz, matches the drone's RC_TICK_MS */
#define NRF24_SEND_TIMEOUT_MS  25    /* must leave room inside TX_PERIOD_MS */

static SemaphoreHandle_t statusMutex;
static RCTx_Status_t     status;
static uint8_t           statusValid = 0;

uint8_t RCTx_GetStatus(RCTx_Status_t *out){
    if(!statusValid) return 0;

    xSemaphoreTake(statusMutex, portMAX_DELAY);
    memcpy(out, &status, sizeof(RCTx_Status_t));
    xSemaphoreGive(statusMutex);
    return 1;
}

static void PublishStatus(const RC_Packet_t *pkt, uint8_t linkOk, uint8_t radioOk){
    xSemaphoreTake(statusMutex, portMAX_DELAY);
    status.lastSent = *pkt;
    status.linkOk   = linkOk;
    status.radioOk  = radioOk;
    xSemaphoreGive(statusMutex);
    statusValid = 1;
}

void RCTxTask(void *argument){
    NRF24_SetTaskHandle(xTaskGetCurrentTaskHandle());
    statusMutex = xSemaphoreCreateMutex();

    JOY_Init();
    uint8_t radioOk = (NRF24_TX_Init() == NRF24_OK);

    RC_Packet_t pkt = {0};
    pkt.magic = RC_PACKET_MAGIC;
    pkt.mode  = RC_MODE_ANGLE;   /* initialization */

    TickType_t nextWake = xTaskGetTickCount();

    while(1){
        JOY_Read(&pkt);
        pkt.mode = ModeSwitch_Read();
        pkt.seq++;

        uint8_t linkOk = 0;
        if(radioOk){
            linkOk = (NRF24_TX_Send(&pkt, NRF24_SEND_TIMEOUT_MS) == NRF24_OK);
        }
        else{
            radioOk = (NRF24_TX_Init() == NRF24_OK);
        }

        PublishStatus(&pkt, linkOk, radioOk);
        vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(TX_PERIOD_MS));
    }
}
