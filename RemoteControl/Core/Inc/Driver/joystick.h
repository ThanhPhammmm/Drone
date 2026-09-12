#ifndef INC_DRIVER_JOYSTICK_H_
#define INC_DRIVER_JOYSTICK_H_

#include "rc_protocol.h"
#include "stm32f4xx_hal.h"

/* ---- Wiring on the TX board (STM32F411CEU6 / Black Pill) ----
 * Mode-2 layout (most common for multirotors):
 *
 *   Left stick  Y (up/down)   -> PB0  (ADC1_IN8)  -> throttle
 *   Left stick  X (left/right)-> PB1  (ADC1_IN9)  -> yaw
 *   Right stick Y (up/down)   -> PA0  (ADC1_IN0)  -> pitch
 *   Right stick X (left/right)-> PA1  (ADC1_IN1)  -> roll
 *
 *   Arm switch  -> PA2, other leg to GND, PA2 configured input pull-up
 *                  (switch closed / pin LOW = armed)
 *
 * Configure in CubeMX: ADC1 with these 4 channels in Scan + Continuous mode,
 * DMA request added (circular, half-word), PA2 as GPIO_Input with pull-up.
 *
 * If any axis moves the wrong way in the ground station / drone logs, it's
 * faster to swap the two wires on the pot than to renumber channels here --
 * but if you'd rather fix it in software, flip the sign in JOY_Read() below.
 */
#define JOY_ADC                 hadc1
#define JOY_NUM_CHANNELS        4

#define JOY_CH_THROTTLE         0   /* PB0 */
#define JOY_CH_YAW              1   /* PB1 */
#define JOY_CH_PITCH            2   /* PA0 */
#define JOY_CH_ROLL             3   /* PA1 */

#define ARM_SW_PORT             GPIOA
#define ARM_SW_PIN              GPIO_PIN_2

void JOY_Init(void);
void JOY_Read(RC_Packet_t *pkt);

#endif /* INC_DRIVER_JOYSTICK_H_ */
