#include "rangeflow_task.h"

#define RANGEFLOW_WAIT_MS          100     /* ToF gives ~30 Hz: a missed edge is picked up here */
#define RANGEFLOW_FLOW_ONLY_MS     20      /* no ToF: nothing wakes the task, read the flow at 50 Hz */

#define RANGEFLOW_POWERUP_MS       50      /* both sensors finish their power-on boot */
#define RANGEFLOW_INIT_TRIES       5
#define RANGEFLOW_INIT_RETRY_MS    50

#define RANGEFLOW_STALL_MS         250     /* no ToF sample for this long: restart it */

static uint32_t lastFlowUs;

static volatile uint8_t rangeReady;
static volatile uint8_t flowReady;

uint8_t RangeFlow_RangeReady(void){ return rangeReady; }
uint8_t RangeFlow_FlowReady(void){ return flowReady; }

static void Range_Restart(void){
    HAL_I2C_DeInit(&VL53L1X_I2C);
    HAL_I2C_Init(&VL53L1X_I2C);
    rangeReady = (VL53L1X_Init() == VL53L1X_OK);
}

/* 1 = a new range sample was read. GPIO1 stays up until the sample is read,
 * so after a missed edge the timeout pass still finds it here. */
static uint8_t Range_Update(uint8_t woken){
    if(!VL53L1X_DataReady() || VL53L1X_Read() != VL53L1X_OK) return 0;

    Range_Data_t range;
    range.distance_m   = vl53l1x.distance_m;
    range.rangeStatus  = vl53l1x.result.rangeStatus;
    range.valid        = vl53l1x.valid;
    range.timestamp_us = woken ? Time_UsAt(vl53l1x.irqCycles) : Time_Us();
    RangeTopic_Publish(&range);
    return 1;
}

static void Flow_Update(void){
    if(PMW3901_Read() != PMW3901_OK) return;

    uint32_t now = Time_Us();
    Flow_Data_t flow;
    flow.deltaX       = pmw3901.deltaX;
    flow.deltaY       = pmw3901.deltaY;
    flow.dt_us        = now - lastFlowUs;
    flow.squal        = pmw3901.squal;
    flow.valid        = pmw3901.valid;
    flow.timestamp_us = now;
    lastFlowUs = now;
    FlowTopic_Publish(&flow);
}

void RangeFlowTask(void *argument){
    VL53L1X_SetTaskHandle(xTaskGetCurrentTaskHandle());
    vTaskDelay(pdMS_TO_TICKS(RANGEFLOW_POWERUP_MS));

    uint8_t rangeOk = 0;
    uint8_t flowOk  = 0;
    for(uint8_t attempt = 0; attempt < RANGEFLOW_INIT_TRIES && !(rangeOk && flowOk); attempt++){
        if(attempt > 0) vTaskDelay(pdMS_TO_TICKS(RANGEFLOW_INIT_RETRY_MS));
        if(!rangeOk) rangeOk = (VL53L1X_Init() == VL53L1X_OK);
        if(!flowOk)  flowOk  = (PMW3901_Init() == PMW3901_OK);
        if(rangeOk && flowOk) break;
    }
    rangeReady = rangeOk;
    flowReady  = flowOk;

    if(!rangeOk && !flowOk){
        vTaskDelete(NULL);
    }

    TickType_t waitTicks = pdMS_TO_TICKS(rangeOk ? RANGEFLOW_WAIT_MS : RANGEFLOW_FLOW_ONLY_MS);
    TickType_t rangeSeen = xTaskGetTickCount();
    lastFlowUs = Time_Us();

    while(1){
        uint8_t woken = (ulTaskNotifyTake(pdTRUE, waitTicks) > 0U);
        TickType_t now = xTaskGetTickCount();

        if(rangeOk){
            if(Range_Update(woken)) rangeSeen = now;
            else if((TickType_t)(now - rangeSeen) >= pdMS_TO_TICKS(RANGEFLOW_STALL_MS)){
                Range_Restart();
                rangeSeen = xTaskGetTickCount();
            }
        }
        if(flowOk) Flow_Update();
    }
}