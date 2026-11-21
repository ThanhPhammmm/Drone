#include "calib_service.h"
#include "calibration.h"
#include "rangeflow_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#define CALSVC_NONE	0xFFU

static uint8_t			haveToken;				/* lastToken is known (since the link came up) */
static uint8_t			lastToken;
static volatile uint8_t pending = CALSVC_NONE;	/* asked for, its task has not taken it yet */
static volatile uint8_t running = CALSVC_NONE;
static volatile uint8_t cancelReq;

static volatile uint8_t lastItem;
static volatile uint8_t state = TLM_CAL_IDLE;
static volatile uint8_t progress;
static volatile uint8_t reason;
static int16_t			result[6];

static uint8_t CalSvc_SensorStatus(Calib_Sensor_t s){
	switch(Calib_GetStatus(s)){
	case CALIB_DONE:
	case CALIB_STORED:		return TLM_CAL_ITEM_OK;
	case CALIB_RUNNING:		return TLM_CAL_ITEM_RUNNING;
	case CALIB_FAILED:
	case CALIB_NO_SENSOR:	return TLM_CAL_ITEM_FAIL;
	default:				return TLM_CAL_ITEM_NONE;
	}
}

static uint8_t CalSvc_Present(uint8_t item){
	switch(item){
	case RC_CAL_IMU:		return Calib_GetStatus(CALIB_SENSOR_BMI088) != CALIB_NO_SENSOR;
	case RC_CAL_BARO:		return Calib_GetStatus(CALIB_SENSOR_BMP388) != CALIB_NO_SENSOR;
	case RC_CAL_COMPASS:	return Calib_GetStatus(CALIB_SENSOR_QMC5883) != CALIB_NO_SENSOR;
	case RC_CAL_FLOW:		return RangeFlow_FlowReady();
	default:				return 0;
	}
}

static void CalSvc_Start(uint8_t item, uint8_t canStart, uint8_t calMode){
	if(item >= RC_CAL_COUNT) return;
	/* the power-on calibrations must be over first */
	uint8_t bootDone = Calib_Finished(CALIB_SENSOR_BMI088) && Calib_Finished(CALIB_SENSOR_BMP388) &&
						Calib_Finished(CALIB_SENSOR_QMC5883);

	taskENTER_CRITICAL();
	if(pending == CALSVC_NONE && running == CALSVC_NONE){
		lastItem = item;
		progress = 0;
		memset(result, 0, sizeof(result));
		cancelReq = 0;
		if(!calMode)					{ state = TLM_CAL_FAILED; reason = TLM_CAL_SWITCH_OFF; }
		else if(!canStart)				{ state = TLM_CAL_FAILED; reason = TLM_CAL_ARMED; }
		else if(!bootDone)				{ state = TLM_CAL_FAILED; reason = TLM_CAL_BUSY; }
		else if(!CalSvc_Present(item))	{ state = TLM_CAL_FAILED; reason = TLM_CAL_NO_SENSOR; }
		else							{ state = TLM_CAL_RUNNING; reason = TLM_CAL_OK; pending = item; }
	}
	taskEXIT_CRITICAL();
}

void CalSvc_Command(uint8_t flags, uint8_t linkOk, uint8_t canStart){
	if(!linkOk){
		haveToken = 0;		/* a remote that restarts must not look like a new request */
		return;
	}
	uint8_t token	= (flags & RC_FLAG_CAL_TOKEN) ? 1U : 0U;
	uint8_t calMode = (flags & RC_FLAG_CAL_MODE) ? 1U : 0U;
	if(!haveToken){
		haveToken = 1;
		lastToken = token;
	}
	else if(token != lastToken){
		lastToken = token;
		CalSvc_Start((flags & RC_FLAG_CAL_ITEM_MASK) >> RC_FLAG_CAL_ITEM_SHIFT, canStart, calMode);
	}

	/* cancel from the button, or the calibration switch went off */
	if((flags & RC_FLAG_CAL_CANCEL) || !calMode){
		taskENTER_CRITICAL();
		if(pending != CALSVC_NONE){
			pending	= CALSVC_NONE;
			state	= TLM_CAL_FAILED;
			reason	= TLM_CAL_CANCELLED;
		}
		else if(running != CALSVC_NONE){
			cancelReq = 1;	/* its task stops and puts the old values back */
		}
		taskEXIT_CRITICAL();
	}
}

uint8_t CalSvc_Take(RC_CalItem_t item){
	if(pending != (uint8_t)item) return 0;	/* polled at up to 1 kHz: no lock in the usual case */
	uint8_t take = 0;
	taskENTER_CRITICAL();
	if(pending == (uint8_t)item){
		pending = CALSVC_NONE;
		running = (uint8_t)item;
		take	= 1;
	}
	taskEXIT_CRITICAL();
	return take;
}

uint8_t CalSvc_CancelRequested(RC_CalItem_t item){
	return running == (uint8_t)item && cancelReq;
}

void CalSvc_Progress(RC_CalItem_t item, uint8_t percent, const int16_t res[6]){
	if(running != (uint8_t)item) return;
	taskENTER_CRITICAL();
	progress = (percent > 100U) ? 100U : percent;
	if(res) memcpy(result, res, sizeof(result));
	taskEXIT_CRITICAL();
}

void CalSvc_Finish(RC_CalItem_t item, uint8_t why, const int16_t res[6]){
	if(running != (uint8_t)item) return;
	taskENTER_CRITICAL();
	state = (why == TLM_CAL_OK) ? TLM_CAL_DONE : TLM_CAL_FAILED;
	reason = why;
	if(why == TLM_CAL_OK) progress = 100;
	if(res) memcpy(result, res, sizeof(result));
	cancelReq = 0;
	running = CALSVC_NONE;
	taskEXIT_CRITICAL();
}

uint8_t CalSvc_Busy(void){
	return pending != CALSVC_NONE || running != CALSVC_NONE;
}

void CalSvc_Snapshot(CalSvc_Snapshot_t *out){
	uint8_t st[RC_CAL_COUNT];
	st[RC_CAL_IMU]		= CalSvc_SensorStatus(CALIB_SENSOR_BMI088);
	st[RC_CAL_BARO]		= CalSvc_SensorStatus(CALIB_SENSOR_BMP388);
	st[RC_CAL_COMPASS]	= CalSvc_SensorStatus(CALIB_SENSOR_QMC5883);
	st[RC_CAL_FLOW]		= !RangeFlow_FlowReady() ? TLM_CAL_ITEM_FAIL : TLM_CAL_ITEM_NONE;

	taskENTER_CRITICAL();
	out->item		= lastItem;
	out->state		= state;
	out->progress	= progress;
	out->reason		= reason;
	memcpy(out->result, result, sizeof(result));
	uint8_t busy	= (pending != CALSVC_NONE) ? pending : running;
	uint8_t flowOk	= (state == TLM_CAL_DONE && lastItem == RC_CAL_FLOW);
	taskEXIT_CRITICAL();

	if(flowOk) st[RC_CAL_FLOW] = TLM_CAL_ITEM_OK;
	if(busy < RC_CAL_COUNT) st[busy] = TLM_CAL_ITEM_RUNNING;
	out->itemStatus = 0;
	for(uint8_t i = 0; i < RC_CAL_COUNT; i++) out->itemStatus |= (uint8_t)(st[i] << (2U * i));
}
