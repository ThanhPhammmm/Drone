#include "rc_tx_task.h"
#include "nrf24_tx.h"
#include "joystick.h"
#include "mode_switch.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define TX_PERIOD_MS			20	/* 50Hz, matches the drone's RC_TICK_MS */
#define NRF24_SEND_TIMEOUT_MS	16	/* 6 tries x ~2.3 ms = ~14 ms; must leave room inside TX_PERIOD_MS */
#define CAL_CANCEL_PACKETS	25	/* cancel is held 0.5 s, so a lost packet does not lose it */

static SemaphoreHandle_t statusMutex;
static RCTx_Status_t status;
static uint8_t statusValid = 0;

/* calibration request (TFT page 6), in the flags of every packet */
static volatile uint8_t calBits;	/* RC_FLAG_CAL_TOKEN + RC_FLAG_CAL_ITEM bits */
static volatile uint8_t calCancelLeft;	/* packets still to carry RC_FLAG_CAL_CANCEL */

void RCTx_CalStart(uint8_t item){
	taskENTER_CRITICAL();
	uint8_t b = (uint8_t)(calBits ^ RC_FLAG_CAL_TOKEN);	/* a flip = one new request */
	b = (uint8_t)((b & ~RC_FLAG_CAL_ITEM_MASK) | ((item << RC_FLAG_CAL_ITEM_SHIFT) & RC_FLAG_CAL_ITEM_MASK));
	calBits = b;
	taskEXIT_CRITICAL();
}

void RCTx_CalCancel(void){
	calCancelLeft = CAL_CANCEL_PACKETS;
}

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
	status.linkOk = linkOk;
	status.radioOk = radioOk;
	xSemaphoreGive(statusMutex);
	statusValid = 1;
}

void RCTxTask(void *argument){
	NRF24_SetTaskHandle(xTaskGetCurrentTaskHandle());

	uint8_t radioOk = (NRF24_TX_Init() == NRF24_OK);
	JOY_Init();
	statusMutex = xSemaphoreCreateMutex();

	uint8_t calSwOn = 0, calSwCount = 0;	/* calibration switch: input with pull-up from CubeMX (rc_tx_task.h) */

	RC_Packet_t pkt = {0};
	pkt.magic = RC_PACKET_MAGIC;
	pkt.mode = RC_MODE_ANGLE;	/* initialization */

	TickType_t nextWake = xTaskGetTickCount();
	uint8_t linkOk = 0;
	uint8_t ack[32];
	uint8_t ackLen = 0;

	while(1){
		JOY_Read(&pkt);
//        pkt.mode = ModeSwitch_Read();
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == GPIO_PIN_RESET){
			pkt.mode = RC_MODE_POS_HOLD;
		}
		else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_RESET){
			pkt.mode = RC_MODE_ALT_HOLD;
		}
		else{
			pkt.mode = RC_MODE_ANGLE;
		}
		//pkt.mode = RC_MODE_ANGLE;
		/* calibration switch, debounced: a bounce must not cancel a running calibration */
		uint8_t calSwNow = (HAL_GPIO_ReadPin(CAL_SW_PORT, CAL_SW_PIN) == GPIO_PIN_RESET) ? 1U : 0U;
		if(calSwNow == calSwOn) calSwCount = 0;
		else if(++calSwCount >= CAL_SW_DEBOUNCE){ calSwOn = calSwNow; calSwCount = 0; }

		pkt.flags |= calBits;	/* JOY_Read set the arm bit only */
		if(calSwOn) pkt.flags |= RC_FLAG_CAL_MODE;
		if(calCancelLeft){
			pkt.flags |= RC_FLAG_CAL_CANCEL;
			calCancelLeft--;
		}
		pkt.seq++;

		if(radioOk){
			linkOk = (NRF24_TX_Send(&pkt, NRF24_SEND_TIMEOUT_MS, ack, &ackLen) == NRF24_OK);
			Telemetry_LinkResult(linkOk);
			if(linkOk && ackLen > 0) Telemetry_Decode(ack, ackLen);
		}
		else{
			radioOk = (NRF24_TX_Init() == NRF24_OK);
			linkOk = 0;
		}

		// For TFT
		PublishStatus(&pkt, linkOk, radioOk);

		vTaskDelayUntil(&nextWake, pdMS_TO_TICKS(TX_PERIOD_MS));
	}
}
