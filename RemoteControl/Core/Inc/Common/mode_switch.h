#ifndef INC_COMMON_MODE_SWITCH_H_
#define INC_COMMON_MODE_SWITCH_H_

#include "rc_protocol.h"
#include "stm32f4xx_hal.h"

/* Two plain on/off switches, decoded by priority into the 3-way mode:
 *
 *   SW_POS_HOLD  SW_ALT_HOLD   ->  mode
 *   ---------------------------------------
 *        ON         x          ->  RC_MODE_POS_HOLD
 *        OFF        ON         ->  RC_MODE_ALT_HOLD
 *        OFF        OFF        ->  RC_MODE_ANGLE
 *
 */

#define MODE_SW_ALT_HOLD_PORT     GPIOB
#define MODE_SW_ALT_HOLD_PIN      GPIO_PIN_14

#define MODE_SW_POS_HOLD_PORT     GPIOB
#define MODE_SW_POS_HOLD_PIN      GPIO_PIN_13

RC_Mode_t ModeSwitch_Read(void);

#endif /* INC_COMMON_MODE_SWITCH_H_ */
