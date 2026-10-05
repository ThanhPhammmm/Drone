#include "calibration.h"
#include "FreeRTOS.h"
#include "task.h"

typedef struct{
	volatile Calib_Status_t status;
	volatile TickType_t     startTick;
	volatile uint8_t        attempts;
} Calib_Entry_t;

static Calib_Entry_t calib[CALIB_SENSOR_COUNT];		/* zeroed = CALIB_WAITING */

void Calib_Start(Calib_Sensor_t s){
	calib[s].startTick = xTaskGetTickCount();
	if(calib[s].attempts < 255U) calib[s].attempts++;
	calib[s].status = CALIB_RUNNING;
}

void Calib_SetStatus(Calib_Sensor_t s, Calib_Status_t status){
	calib[s].status = status;
}

Calib_Status_t Calib_GetStatus(Calib_Sensor_t s){
	return calib[s].status;
}

uint32_t Calib_ElapsedMs(Calib_Sensor_t s){
	if(calib[s].status != CALIB_RUNNING) return 0;
	return (uint32_t)(xTaskGetTickCount() - calib[s].startTick) * portTICK_PERIOD_MS;
}

uint8_t Calib_Attempts(Calib_Sensor_t s){
	return calib[s].attempts;
}

uint8_t Calib_Finished(Calib_Sensor_t s){
	Calib_Status_t st = calib[s].status;
	return st == CALIB_DONE || st == CALIB_STORED || st == CALIB_FAILED || st == CALIB_NO_SENSOR;
}

uint8_t Calib_Usable(Calib_Sensor_t s){
	Calib_Status_t st = calib[s].status;
	return st == CALIB_DONE || st == CALIB_STORED;
}

uint8_t Calib_IsTurn(Calib_Sensor_t s){
	for(uint8_t k = 0; k < (uint8_t)s; k++){
		if(!Calib_Finished((Calib_Sensor_t)k)) return 0;
	}
	return 1;
}

/* IMU and baro are needed to fly. The compass is not (nothing flown uses the
 * heading yet), so a failed compass calibration does not block arming -- but
 * arming still waits for the sequence to end, so the drone is never armed
 * while it is supposed to be turned by hand. */
uint8_t Calib_ReadyToArm(void){
	return Calib_Usable(CALIB_SENSOR_BMI088) &&
	       Calib_Usable(CALIB_SENSOR_BMP388) &&
	       Calib_Finished(CALIB_SENSOR_QMC5883);
}

const char *Calib_StatusName(Calib_Status_t status){
	switch(status){
	case CALIB_WAITING:   return "WAITING";
	case CALIB_RUNNING:   return "RUNNING";
	case CALIB_DONE:      return "DONE";
	case CALIB_STORED:    return "STORED";
	case CALIB_FAILED:    return "FAILED";
	case CALIB_NO_SENSOR: return "NO_SENSOR";
	default:              return "?";
	}
}
