#include "vl53l1x.h"
#include "semphr.h"

#define VL53L1X_I2C_TIMEOUT_MS      25      /* the 91-byte config block takes ~9 ms at 100 kHz */
#define VL53L1X_BOOT_TIMEOUT_MS     100
#define VL53L1X_FIRST_DATA_MS       200     /* first ranging of the init (VHV calibration) */

VL53L1X_Handle_t vl53l1x;

static SemaphoreHandle_t vl53l1xDmaSem = NULL;

/* 0x2D .. 0x87, values for 2.8 V / 3.3 V pull-ups on I2C and GPIO1 */
static const uint8_t VL53L1X_DefaultConfig[VL53L1X_DEFAULT_CONFIG_LEN] = {
    0x00, /* 0x2d : fast plus mode (1 MHz I2C) off */
    0x01, /* 0x2e : I2C pulled up at AVDD (2.8 / 3.3 V) */
    0x01, /* 0x2f : GPIO pulled up at AVDD */
    0x11, /* 0x30 : bit 4 = 1: GPIO1 active low  */
    0x02, /* 0x31 */
    0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01, 0x01,             /* 0x32 .. 0x39 */
    0x00, 0x00, 0x00, 0x00, 0xff, 0x00, 0x0F, 0x00,             /* 0x3a .. 0x41 */
    0x00, 0x00, 0x00, 0x00,                                     /* 0x42 .. 0x45 */
    0x20, /* 0x46 : interrupt on new sample ready */
    0x0b, 0x00, 0x00, 0x02, 0x0a, 0x21, 0x00, 0x00, 0x05,       /* 0x47 .. 0x4f */
    0x00, 0x00, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x38,             /* 0x50 .. 0x57 */
    0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x01, 0xcc,             /* 0x58 .. 0x5f */
    0x0f, 0x01, 0xf1, 0x0d,                                     /* 0x60 .. 0x63 */
    0x01, 0x68, /* 0x64, 0x65 : sigma threshold, 90 mm */
    0x00, 0x80, /* 0x66, 0x67 : min signal rate */
    0x08, 0xb8, 0x00, 0x00,                                     /* 0x68 .. 0x6b */
    0x00, 0x00, 0x0f, 0x89, /* 0x6c .. 0x6f : inter-measurement period */
    0x00, 0x00,                                                 /* 0x70, 0x71 */
    0x00, 0x00, 0x00, 0x00, /* 0x72 .. 0x75 : distance thresholds */
    0x00, 0x01, 0x0f, 0x0d, 0x0e, 0x0e, 0x00, 0x00, 0x02,       /* 0x76 .. 0x7e */
    0xc7, /* 0x7f : ROI centre */
    0xff, /* 0x80 : ROI 16 x 16 */
    0x9B, 0x00, 0x00, 0x00, 0x01,                               /* 0x81 .. 0x85 */
    0x00, /* 0x86 : clear interrupt */
    0x00  /* 0x87 : ranging stopped */
};

static const uint8_t VL53L1X_StatusMap[24] = {
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0,
    255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
    255, 255, 11, 12
};

static const uint16_t VL53L1X_BudgetMs[7]    = { 15,     20,     33,     50,     100,    200,    500    };
static const uint16_t VL53L1X_ShortA[7]      = { 0x001D, 0x0051, 0x00D6, 0x01AE, 0x02E1, 0x03E1, 0x0591 };
static const uint16_t VL53L1X_ShortB[7]      = { 0x0027, 0x006E, 0x006E, 0x01E8, 0x0388, 0x0496, 0x05C1 };
static const uint16_t VL53L1X_LongA[7]       = { 0,      0x001E, 0x0060, 0x00AD, 0x01CC, 0x02D9, 0x048F };
static const uint16_t VL53L1X_LongB[7]       = { 0,      0x0022, 0x006E, 0x00C6, 0x01EA, 0x02F8, 0x04A4 };

void VL53L1X_SetTaskHandle(TaskHandle_t handle){
    vl53l1x.task = handle;
}

void VL53L1X_IRQ_Handler(void){
    vl53l1x.irqCycles = DWT->CYCCNT;
    if(vl53l1x.task == NULL) return;

    BaseType_t hpw = pdFALSE;
    vTaskNotifyGiveFromISR(vl53l1x.task, &hpw);
    portYIELD_FROM_ISR(hpw);
}

void VL53L1X_DMA_Callback(I2C_HandleTypeDef *hi2c){
    if(vl53l1xDmaSem == NULL) return;

    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(vl53l1xDmaSem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void VL53L1X_ClearDMACompletion(void){
    while(xSemaphoreTake(vl53l1xDmaSem, 0) == pdTRUE){}
}

static VL53L1X_Status_t VL53L1X_WaitDMA(HAL_StatusTypeDef started){
    if(started == HAL_OK &&
       xSemaphoreTake(vl53l1xDmaSem, pdMS_TO_TICKS(VL53L1X_I2C_TIMEOUT_MS)) == pdTRUE &&
       HAL_I2C_GetError(&VL53L1X_I2C) == HAL_I2C_ERROR_NONE){
        return VL53L1X_OK;
    }
    HAL_I2C_DeInit(&VL53L1X_I2C);
    HAL_I2C_Init(&VL53L1X_I2C);
    return VL53L1X_ERROR;
}

static VL53L1X_Status_t VL53L1X_WriteRegs(uint16_t reg, const uint8_t *buf, uint16_t len){
    VL53L1X_ClearDMACompletion();
    return VL53L1X_WaitDMA(HAL_I2C_Mem_Write_IT(&VL53L1X_I2C, VL53L1X_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
                                                (uint8_t *)buf, len));
}

static VL53L1X_Status_t VL53L1X_ReadRegs(uint16_t reg, uint8_t *buf, uint16_t len){
    if(len == 1U){
        uint8_t two[2];
        if(VL53L1X_ReadRegs(reg, two, 2) != VL53L1X_OK) return VL53L1X_ERROR;
        buf[0] = two[0];
        return VL53L1X_OK;
    }
    VL53L1X_ClearDMACompletion();
    return VL53L1X_WaitDMA(HAL_I2C_Mem_Read_DMA(&VL53L1X_I2C, VL53L1X_I2C_ADDR, reg, I2C_MEMADD_SIZE_16BIT,
                                                buf, len));
}

static VL53L1X_Status_t VL53L1X_WriteByte(uint16_t reg, uint8_t val){
    return VL53L1X_WriteRegs(reg, &val, 1);
}

static VL53L1X_Status_t VL53L1X_WriteWord(uint16_t reg, uint16_t val){
    uint8_t b[2] = { (uint8_t)(val >> 8), (uint8_t)val };
    return VL53L1X_WriteRegs(reg, b, 2);
}

static VL53L1X_Status_t VL53L1X_WriteDWord(uint16_t reg, uint32_t val){
    uint8_t b[4] = { (uint8_t)(val >> 24), (uint8_t)(val >> 16), (uint8_t)(val >> 8), (uint8_t)val };
    return VL53L1X_WriteRegs(reg, b, 4);
}

static VL53L1X_Status_t VL53L1X_ReadByte(uint16_t reg, uint8_t *val){
    return VL53L1X_ReadRegs(reg, val, 1);
}

static VL53L1X_Status_t VL53L1X_ReadWord(uint16_t reg, uint16_t *val){
    uint8_t b[2];
    if(VL53L1X_ReadRegs(reg, b, 2) != VL53L1X_OK) return VL53L1X_ERROR;
    *val = (uint16_t)((b[0] << 8) | b[1]);
    return VL53L1X_OK;
}

static VL53L1X_Status_t VL53L1X_ClearInterrupt(void){
    return VL53L1X_WriteByte(VL53L1X_REG_SYSTEM_INTERRUPT_CLEAR, VL53L1X_INTERRUPT_CLEAR);
}

VL53L1X_Status_t VL53L1X_StartRanging(void){
    return VL53L1X_WriteByte(VL53L1X_REG_SYSTEM_MODE_START, VL53L1X_MODE_START_RANGING);
}

VL53L1X_Status_t VL53L1X_StopRanging(void){
    return VL53L1X_WriteByte(VL53L1X_REG_SYSTEM_MODE_START, VL53L1X_MODE_STOP_RANGING);
}

uint8_t VL53L1X_DataReady(void){
    uint8_t status = 0;
    if(VL53L1X_ReadByte(VL53L1X_REG_GPIO_TIO_HV_STATUS, &status) != VL53L1X_OK) return 0;
    return ((status & 0x01) == vl53l1x.intActiveHigh) ? 1 : 0;
}

VL53L1X_Status_t VL53L1X_SetTimingBudget(uint16_t ms){
    for(uint8_t i = 0; i < 7; i++){
        if(VL53L1X_BudgetMs[i] != ms) continue;

        uint16_t a, b;
        if(vl53l1x.distanceMode == VL53L1X_DISTANCE_SHORT){
            a = VL53L1X_ShortA[i];
            b = VL53L1X_ShortB[i];
        }
        else{
            a = VL53L1X_LongA[i];
            b = VL53L1X_LongB[i];
        }
        if(a == 0) return VL53L1X_CONFIG_ERROR;     /* 15 ms: short mode only */

        if(VL53L1X_WriteWord(VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_A_HI, a) != VL53L1X_OK) return VL53L1X_ERROR;
        if(VL53L1X_WriteWord(VL53L1X_REG_RANGE_CONFIG_TIMEOUT_MACROP_B_HI, b) != VL53L1X_OK) return VL53L1X_ERROR;
        vl53l1x.timingBudgetMs = ms;
        return VL53L1X_OK;
    }
    return VL53L1X_CONFIG_ERROR;
}

VL53L1X_Status_t VL53L1X_SetDistanceMode(VL53L1X_DistanceMode_t mode){
    uint8_t  phasecal, vcselA, vcselB, validPhase;
    uint16_t woi, initialPhase;

    if(mode == VL53L1X_DISTANCE_SHORT){
        phasecal = 0x14; vcselA = 0x07; vcselB = 0x05; validPhase = 0x38;
        woi = 0x0705; initialPhase = 0x0606;
    }
    else if(mode == VL53L1X_DISTANCE_LONG){
        phasecal = 0x0A; vcselA = 0x0F; vcselB = 0x0D; validPhase = 0xB8;
        woi = 0x0F0D; initialPhase = 0x0E0E;
    }
    else return VL53L1X_CONFIG_ERROR;

    if(VL53L1X_WriteByte(VL53L1X_REG_PHASECAL_CONFIG_TIMEOUT_MACROP, phasecal)  != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteByte(VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_A, vcselA)       != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteByte(VL53L1X_REG_RANGE_CONFIG_VCSEL_PERIOD_B, vcselB)       != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteByte(VL53L1X_REG_RANGE_CONFIG_VALID_PHASE_HIGH, validPhase) != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteWord(VL53L1X_REG_SD_CONFIG_WOI_SD0, woi)                    != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteWord(VL53L1X_REG_SD_CONFIG_INITIAL_PHASE_SD0, initialPhase) != VL53L1X_OK) return VL53L1X_ERROR;

    vl53l1x.distanceMode = mode;
    if(vl53l1x.timingBudgetMs == 0) return VL53L1X_OK;
    return VL53L1X_SetTimingBudget(vl53l1x.timingBudgetMs);
}

VL53L1X_Status_t VL53L1X_SetInterMeasurement(uint32_t ms){
    uint16_t clockPll = 0;
    if(VL53L1X_ReadWord(VL53L1X_REG_RESULT_OSC_CALIBRATE_VAL, &clockPll) != VL53L1X_OK) return VL53L1X_ERROR;
    clockPll &= 0x3FF;
    uint32_t period = (uint32_t)((float)clockPll * (float)ms * 1.075f);
    return VL53L1X_WriteDWord(VL53L1X_REG_SYSTEM_INTERMEASUREMENT_PERIOD, period);
}

VL53L1X_Status_t VL53L1X_Init(void){
    if(vl53l1xDmaSem == NULL) vl53l1xDmaSem = xSemaphoreCreateBinary();
    if(vl53l1xDmaSem == NULL) return VL53L1X_ERROR;

    vl53l1x.valid          = 0;
    vl53l1x.distanceMode   = VL53L1X_DISTANCE_LONG;    /* what the default block sets */
    vl53l1x.timingBudgetMs = 0;

    /* boot: the firmware sets bit 0 when it is ready; until then it may NACK */
    uint8_t booted = 0;
    for(uint16_t t = 0; t < VL53L1X_BOOT_TIMEOUT_MS && !booted; t += 2){
        uint8_t state = 0;
        if(VL53L1X_ReadByte(VL53L1X_REG_FIRMWARE_SYSTEM_STATUS, &state) == VL53L1X_OK && (state & 0x01)){
            booted = 1;
        }
        else{
            vTaskDelay(pdMS_TO_TICKS(2));
        }
    }
    if(!booted) return VL53L1X_TIMEOUT;

    uint16_t id = 0;
    if(VL53L1X_ReadWord(VL53L1X_REG_IDENTIFICATION_MODEL_ID, &id) != VL53L1X_OK) return VL53L1X_ERROR;
    if(id != VL53L1X_MODEL_ID_VALUE) return VL53L1X_INVALID_CHIP;

    if(VL53L1X_WriteRegs(VL53L1X_DEFAULT_CONFIG_START, VL53L1X_DefaultConfig, VL53L1X_DEFAULT_CONFIG_LEN) != VL53L1X_OK){
        return VL53L1X_ERROR;
    }

    uint8_t mux = 0;
    if(VL53L1X_ReadByte(VL53L1X_REG_GPIO_HV_MUX_CTRL, &mux) != VL53L1X_OK) return VL53L1X_ERROR;
    vl53l1x.intActiveHigh = (mux & VL53L1X_GPIO_HV_MUX_ACTIVE_LOW) ? 0 : 1;

    if(VL53L1X_StartRanging() != VL53L1X_OK) return VL53L1X_ERROR;
    uint8_t ready = 0;
    for(uint16_t t = 0; t < VL53L1X_FIRST_DATA_MS && !ready; t += 2){
        ready = VL53L1X_DataReady();
        if(!ready) vTaskDelay(pdMS_TO_TICKS(2));
    }
    if(!ready) return VL53L1X_TIMEOUT;
    if(VL53L1X_ClearInterrupt() != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_StopRanging()    != VL53L1X_OK) return VL53L1X_ERROR;

    if(VL53L1X_WriteByte(VL53L1X_REG_VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND, 0x09) != VL53L1X_OK) return VL53L1X_ERROR;
    if(VL53L1X_WriteByte(VL53L1X_REG_VHV_CONFIG_INIT, 0x00) != VL53L1X_OK) return VL53L1X_ERROR;

    VL53L1X_Status_t st;
    if((st = VL53L1X_SetDistanceMode(VL53L1X_DEFAULT_MODE))         != VL53L1X_OK) return st;
    if((st = VL53L1X_SetTimingBudget(VL53L1X_DEFAULT_BUDGET_MS))    != VL53L1X_OK) return st;
    if((st = VL53L1X_SetInterMeasurement(VL53L1X_DEFAULT_BUDGET_MS)) != VL53L1X_OK) return st;

    if(VL53L1X_ClearInterrupt() != VL53L1X_OK) return VL53L1X_ERROR;
    return VL53L1X_StartRanging();
}

VL53L1X_Status_t VL53L1X_Read(void){
    uint8_t b[VL53L1X_RESULT_LEN];
    if(VL53L1X_ReadRegs(VL53L1X_REG_RESULT_RANGE_STATUS, b, VL53L1X_RESULT_LEN) != VL53L1X_OK){
        return VL53L1X_ERROR;
    }

    uint8_t raw = b[0] & 0x1F;
    VL53L1X_Result_t *r = &vl53l1x.result;
    r->rangeStatus   = (raw < 24) ? VL53L1X_StatusMap[raw] : VL53L1X_RANGE_NO_UPDATE;
    r->numSpads      = b[3];
    r->ambient       = (uint16_t)(((b[7] << 8) | b[8]) * 8);
    r->distance_mm   = (uint16_t)((b[13] << 8) | b[14]);
    r->signalPerSpad = (uint16_t)(((b[15] << 8) | b[16]) * 8);

    vl53l1x.distance_m = (float)r->distance_mm * 0.001f;
    vl53l1x.valid      = (r->rangeStatus == VL53L1X_RANGE_VALID);

    return VL53L1X_ClearInterrupt();
}
