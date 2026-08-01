#ifndef INC_RC_PROTOCOL_H_
#define INC_RC_PROTOCOL_H_

#include <stdint.h>


#define RC_PACKET_MAGIC     0xA5

#define RC_CHANNEL_MAX      1000        /* sticks:   -1000..+1000 */
#define RC_THROTTLE_MAX     1000        /* throttle:     0..1000  */

typedef enum{
    RC_MODE_RATE = 0,
    RC_MODE_ANGLE,
    RC_MODE_ALT_HOLD,
    RC_MODE_COUNT
} RC_Mode_t;

#define RC_FLAG_ARM         0x01

typedef struct __attribute__((packed)){
    uint8_t  magic;
    uint8_t  seq;
    int16_t  roll;
    int16_t  pitch;
    int16_t  yaw;
    uint16_t throttle;
    uint8_t  mode;
    uint8_t  flags;
} RC_Packet_t;              /* 12 bytes */

#endif /* INC_RC_PROTOCOL_H_ */
