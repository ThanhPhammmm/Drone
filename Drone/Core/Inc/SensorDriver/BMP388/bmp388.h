#ifndef INC_SENSORDRIVER_BMP388_BMP388_H_
#define INC_SENSORDRIVER_BMP388_BMP388_H_

#include "bmp388_data_types.h"
#include "bmp388_reg.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define BMP388_INT_PIN GPIO_PIN_8

typedef struct{
    BMP388_Raw_t       raw;
    BMP388_Calib_t     calib;
    BMP388_GroundRef_t groundRef;

    float    pressure_pa; // pascal 
    float    temperature_c; // celsius
    float    altitude_m; // m
    uint8_t  addr;

    TaskHandle_t      task;
    volatile uint32_t irqCycles;    /* DWT->CYCCNT at the last INT edge */
} BMP388_Handle_t;

extern BMP388_Handle_t bmp388;

BMP388_Status_t BMP388_Init(void);
uint8_t BMP388_DataReady(void);
BMP388_Status_t BMP388_Read(void);
void BMP388_GroundRefReset(void);
void BMP388_GroundRefAccumulate(void);
void BMP388_GroundRefSet(float pressure_pa);
void BMP388_SetTaskHandle(TaskHandle_t handle);
void BMP388_IRQ_Handler(void);
void BMP388_DMA_Callback(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSORDRIVER_BMP388_BMP388_H_ */