#ifndef INC_STATE_FLIGHT_MODE_H_
#define INC_STATE_FLIGHT_MODE_H_

#include <stdint.h>

typedef enum{
	FLIGHT_MODE_ANGLE = 0,
	FLIGHT_MODE_ALT_HOLD,
	FLIGHT_MODE_POS_HOLD,		/* TODO: ALT_HOLD + horizontal position hold, see flight_mode.c */
} FlightMode_t;

FlightMode_t FlightMode_Resolve(uint8_t requested, uint8_t altitudeValid);

/* Throttle stick (0..1) interpretation, per mode. */
uint8_t FlightMode_TakeoffRequested(FlightMode_t mode, float throttle);
uint8_t FlightMode_DescendRequested(FlightMode_t mode, float throttle);
float FlightMode_ClimbRate(float throttle);	/* ALT_HOLD: m/s, +up */
void FlightMode_SetActive(FlightMode_t mode);
FlightMode_t FlightMode_GetActive(void);

#endif /* INC_STATE_FLIGHT_MODE_H_ */
