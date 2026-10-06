#include "flight_mode.h"
#include "rc_protocol.h"

#define FM_ANGLE_TAKEOFF_THROTTLE	0.08f	/* ANGLE: stick above this = take-off begins */
#define FM_ANGLE_DESCEND_THROTTLE	0.05f	/* ANGLE: stick below this = landing / abort take-off */

#define FM_ALT_HOLD_DEADBAND		0.10f	/* fraction of the +-1 range around centre that means "hold" */
#define FM_MAX_CLIMB_RATE			2.0f	/* m/s at full stick deflection */

/*
 * POS_HOLD -- not implemented yet, it falls back to ALT_HOLD.
 */
static volatile FlightMode_t activeMode = FLIGHT_MODE_ANGLE;

FlightMode_t FlightMode_Resolve(uint8_t requested, uint8_t altitudeValid){
	switch(requested){
	case RC_MODE_POS_HOLD:		/* TODO: FLIGHT_MODE_POS_HOLD when a position estimate exists */
	case RC_MODE_ALT_HOLD:
		return altitudeValid ? FLIGHT_MODE_ALT_HOLD : FLIGHT_MODE_ANGLE;

	case RC_MODE_ANGLE:
	default:
		return FLIGHT_MODE_ANGLE;
	}
}

float FlightMode_ClimbRate(float throttle){
	float t = (throttle - 0.5f) * 2.0f;		/* -1..+1 around centre */

	if(t > -FM_ALT_HOLD_DEADBAND && t < FM_ALT_HOLD_DEADBAND) return 0.0f;

	if(t > 0.0f) t = (t - FM_ALT_HOLD_DEADBAND) / (1.0f - FM_ALT_HOLD_DEADBAND);
	else         t = (t + FM_ALT_HOLD_DEADBAND) / (1.0f - FM_ALT_HOLD_DEADBAND);

	if(t >  1.0f) t =  1.0f;
	if(t < -1.0f) t = -1.0f;

	return t * FM_MAX_CLIMB_RATE;
}

uint8_t FlightMode_TakeoffRequested(FlightMode_t mode, float throttle){
	if(mode == FLIGHT_MODE_ANGLE) return throttle > FM_ANGLE_TAKEOFF_THROTTLE;
	return FlightMode_ClimbRate(throttle) > 0.0f;		/* ALT_HOLD / POS_HOLD: stick above centre */
}

uint8_t FlightMode_DescendRequested(FlightMode_t mode, float throttle){
	if(mode == FLIGHT_MODE_ANGLE) return throttle < FM_ANGLE_DESCEND_THROTTLE;
	return FlightMode_ClimbRate(throttle) < 0.0f;		/* ALT_HOLD / POS_HOLD: stick below centre */
}

void FlightMode_SetActive(FlightMode_t mode){
	activeMode = mode;
}

FlightMode_t FlightMode_GetActive(void){
	return activeMode;
}