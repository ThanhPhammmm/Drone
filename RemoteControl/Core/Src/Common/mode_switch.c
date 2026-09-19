#include "mode_switch.h"

static inline uint8_t SwitchOn(GPIO_TypeDef *port, uint16_t pin){
    return (HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET);   /* closed to GND = ON */
}

RC_Mode_t ModeSwitch_Read(void){
    if(SwitchOn(MODE_SW_POS_HOLD_PORT, MODE_SW_POS_HOLD_PIN)){
        return RC_MODE_POS_HOLD;
    }
    else if(SwitchOn(MODE_SW_ALT_HOLD_PORT, MODE_SW_ALT_HOLD_PIN)){
        return RC_MODE_ALT_HOLD;
    }
    else{
        return RC_MODE_ANGLE;
    }
}
