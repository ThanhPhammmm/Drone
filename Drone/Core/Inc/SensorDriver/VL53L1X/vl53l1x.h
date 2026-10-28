#ifndef INC_SENSORDRIVER_VL53L1X_VL53L1X_H_
#define INC_SENSORDRIVER_VL53L1X_VL53L1X_H_

#include "vl53l1x_data_types.h"
#include "vl53l1x_reg.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"

#define VL53L1X_I2C              hi2c1
#define VL53L1X_INT_PIN          VL53L1X_GPIO1_Pin

/* chosen at init; 33 ms in long mode = ~30 Hz, up to ~4 m indoors */
#define VL53L1X_DEFAULT_MODE         VL53L1X_DISTANCE_LONG
#define VL53L1X_DEFAULT_BUDGET_MS    33

typedef struct{
    VL53L1X_Result_t result;
    float    distance_m;
    uint8_t  valid;

    VL53L1X_DistanceMode_t distanceMode;
    uint16_t timingBudgetMs;
    uint8_t  intActiveHigh;

    TaskHandle_t      task;
    volatile uint32_t irqCycles;
} VL53L1X_Handle_t;

extern VL53L1X_Handle_t vl53l1x;

VL53L1X_Status_t VL53L1X_Init(void);    /* ends with the sensor ranging */
VL53L1X_Status_t VL53L1X_SetDistanceMode(VL53L1X_DistanceMode_t mode);
VL53L1X_Status_t VL53L1X_SetTimingBudget(uint16_t ms);     /* 15 (short only), 20, 33, 50, 100, 200, 500 */
VL53L1X_Status_t VL53L1X_SetInterMeasurement(uint32_t ms); /* >= the timing budget */
VL53L1X_Status_t VL53L1X_StartRanging(void);
VL53L1X_Status_t VL53L1X_StopRanging(void);
uint8_t          VL53L1X_DataReady(void);
VL53L1X_Status_t VL53L1X_Read(void);
void VL53L1X_SetTaskHandle(TaskHandle_t handle);
void VL53L1X_IRQ_Handler(void);
void VL53L1X_DMA_Callback(I2C_HandleTypeDef *hi2c);

#endif /* INC_SENSORDRIVER_VL53L1X_VL53L1X_H_ */
