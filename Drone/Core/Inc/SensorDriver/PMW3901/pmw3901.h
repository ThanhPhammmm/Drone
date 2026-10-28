#ifndef INC_SENSORDRIVER_PMW3901_PMW3901_H_
#define INC_SENSORDRIVER_PMW3901_PMW3901_H_

#include "pmw3901_data_types.h"
#include "pmw3901_reg.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "task.h"

#define PMW3901_SPI             hspi3
#define PMW3901_CS_PORT         PMW3901_CS_GPIO_Port
#define PMW3901_CS_PIN          PMW3901_CS_Pin

#define PMW3901_SQUAL_MIN       0x19    /* below this the floor has too few features to trust */

#define PMW3901_RAD_PER_COUNT   (1.0f / 500.0f)

typedef struct{
    PMW3901_Burst_t burst;
    int16_t  deltaX;
    int16_t  deltaY;
    uint8_t  squal;
    uint8_t  valid;
} PMW3901_Handle_t;

extern PMW3901_Handle_t pmw3901;

PMW3901_Status_t PMW3901_Init(void);
PMW3901_Status_t PMW3901_Read(void);
void PMW3901_TxCallback(SPI_HandleTypeDef *hspi);

#endif /* INC_SENSORDRIVER_PMW3901_PMW3901_H_ */
