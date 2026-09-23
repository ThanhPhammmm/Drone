#ifndef INC_DRIVER_JOYSTICK_H_
#define INC_DRIVER_JOYSTICK_H_

#include "rc_protocol.h"
#include "stm32f4xx_hal.h"

/* ---- Wiring on the TX board (STM32F411CEU6 / Black Pill) ----
 * Mode-2 layout (most common for multirotors):
 *
 *   Left stick  X (up/down)   -> PA1  (ADC1_IN1)  -> throttle
 *   Left stick  Y (left/right)-> PA2  (ADC1_IN2)  -> yaw
 *   Right stick X (up/down)   -> PA3  (ADC1_IN3)  -> pitch
 *   Right stick Y (left/right)-> PA4  (ADC1_IN4)  -> roll
 *
 *   Arm switch  -> PB7, other leg to GND, PB7 configured input pull-up
 *                  (switch closed / pin LOW = armed)
 *
 * Configure in CubeMX: ADC1 with these 4 channels in Scan + Continuous mode,
 * DMA request added (circular, half-word), PB7 as GPIO_Input with pull-up.
 *
 * If any axis moves the wrong way in the ground station / drone logs, it's
 * faster to swap the two wires on the pot than to renumber channels here --
 * but if you'd rather fix it in software, flip the sign in JOY_Read() below.
 */
#define JOY_ADC                 hadc1
#define JOY_NUM_CHANNELS        4

#define JOY_CH_THROTTLE         0   /* PA2 */
#define JOY_CH_YAW              1   /* PA1 */
#define JOY_CH_PITCH            2   /* PA4 */
#define JOY_CH_ROLL             3   /* PA3 */

#define ARM_SW_PORT             GPIOB
#define ARM_SW_PIN              GPIO_PIN_10

void JOY_Init(void);
void JOY_Read(RC_Packet_t *pkt);
void JOY_DMA_Callback(ADC_HandleTypeDef *hadc1);

#endif /* INC_DRIVER_JOYSTICK_H_ */
