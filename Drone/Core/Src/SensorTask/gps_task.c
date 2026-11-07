#include "gps_task.h"

#define GPS_POWERUP_MS          1000U   /* the receiver boots */
#define GPS_POLL_MS             10U     /* well inside the 44 ms the DMA buffer holds */
#define GPS_PORT_SWITCH_MS      100U    /* after CFG-PRT the receiver moves to the new baud */
#define GPS_FIRST_PVT_MS        1500U   /* after the setup: this long for the first NAV-PVT */
#define GPS_LOST_MS             2000U   /* no NAV-PVT this long: find the receiver again */
#define GPS_RETRY_MS            1000U

/* Bauds the receiver may be at: ours if it kept running since the last
 * start, 9600 from the factory, the others some boards come set to. */
static const uint32_t gpsBauds[] = { NEO_M8N_BAUD, 9600U, 38400U, 57600U };

static volatile uint8_t gpsFound;

uint8_t Gps_Found(void){
	return gpsFound;
}

static void Gps_Publish(void){
	const NEO_M8N_Pvt_t *v = &neoM8n.pvt;
	Gps_Data_t g;
	g.fixType      = v->fixType;
	g.fixOk        = v->fixOk;
	g.numSV        = v->numSV;
	g.lat          = v->lat;
	g.lon          = v->lon;
	g.altMsl       = (float)v->hMSL * 0.001f;
	g.hAcc         = (float)v->hAcc * 0.001f;
	g.vAcc         = (float)v->vAcc * 0.001f;
	g.velN         = (float)v->velN * 0.001f;
	g.velE         = (float)v->velE * 0.001f;
	g.velD         = (float)v->velD * 0.001f;
	g.sAcc         = (float)v->sAcc * 0.001f;
	g.pDop         = (float)v->pDOP * 0.01f;
	g.iTOW         = v->iTOW;
	g.timestamp_us = Time_Us();
	GpsTopic_Publish(&g);
}

static uint8_t Gps_WaitPvt(uint32_t ms){
	TickType_t start = xTaskGetTickCount();
	while((TickType_t)(xTaskGetTickCount() - start) < pdMS_TO_TICKS(ms)){
		vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
		if(NEO_M8N_Poll()) return 1;
	}
	return 0;
}

/* At each baud the receiver may be at: tell it to move to NEO_M8N_BAUD with
 * UBX only, follow it there, send the setup and wait for a NAV-PVT. */
static uint8_t Gps_Find(void){
	for(uint8_t i = 0; i < sizeof(gpsBauds) / sizeof(gpsBauds[0]); i++){
		if(NEO_M8N_StartRx(gpsBauds[i]) != NEO_M8N_OK) return 0;
		(void)NEO_M8N_SetPort();
		vTaskDelay(pdMS_TO_TICKS(GPS_PORT_SWITCH_MS));

		if(NEO_M8N_StartRx(NEO_M8N_BAUD) != NEO_M8N_OK) return 0;
		(void)NEO_M8N_Configure();
		if(Gps_WaitPvt(GPS_FIRST_PVT_MS)) return 1;
	}
	return 0;
}

void GpsTask(void *argument){
	vTaskDelay(pdMS_TO_TICKS(GPS_POWERUP_MS));

	TickType_t lastPvt = xTaskGetTickCount();

	while(1){
		if(!gpsFound){
			if(!Gps_Find()){
				vTaskDelay(pdMS_TO_TICKS(GPS_RETRY_MS));
				continue;
			}
			gpsFound = 1;
			lastPvt  = xTaskGetTickCount();
			Gps_Publish();
			continue;
		}

		vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));

		if(NEO_M8N_Poll()){
			lastPvt = xTaskGetTickCount();
			Gps_Publish();
		}
		else if((TickType_t)(xTaskGetTickCount() - lastPvt) > pdMS_TO_TICKS(GPS_LOST_MS)){
			gpsFound = 0;               /* unplugged, or restarted at its own baud */
		}
	}
}
