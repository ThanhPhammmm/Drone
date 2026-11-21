#ifndef INC_COMMON_CALIB_SERVICE_H_
#define INC_COMMON_CALIB_SERVICE_H_

#include <stdint.h>
#include "rc_protocol.h"

typedef struct{
	uint8_t item;
	uint8_t state;
	uint8_t progress;		/* % */
	uint8_t reason;			/* TLM_CAL_OK .. */
	uint8_t itemStatus;
	int16_t result[6];
} CalSvc_Snapshot_t;

void CalSvc_Command(uint8_t flags, uint8_t linkOk, uint8_t canStart);
uint8_t CalSvc_Take(RC_CalItem_t item);
uint8_t CalSvc_CancelRequested(RC_CalItem_t item);
void CalSvc_Progress(RC_CalItem_t item, uint8_t percent, const int16_t result[6]);
void CalSvc_Finish(RC_CalItem_t item, uint8_t reason, const int16_t result[6]);     /* TLM_CAL_OK = done */
uint8_t CalSvc_Busy(void);
void CalSvc_Snapshot(CalSvc_Snapshot_t *out);

#endif /* INC_COMMON_CALIB_SERVICE_H_ */
