#include "pmw3901.h"
#include "semphr.h"
#include "timebase.h"
#include <string.h>

#define PMW3901_SPI_TIMEOUT_MS      5
#define PMW3901_ADDR_DATA_US        50      /* after the address byte, before data */
#define PMW3901_CS_HOLD_US          50      /* after the last byte, before CS goes high */
#define PMW3901_BETWEEN_US          200     /* CS high between two accesses */

PMW3901_Handle_t pmw3901;

static SemaphoreHandle_t pmw3901DmaSem = NULL;
static uint8_t pmw3901Zeros[PMW3901_BURST_LEN];

typedef struct{
    uint8_t reg;
    uint8_t val;
} PMW3901_RegVal_t;

/* PixArt's performance settings, written after the power-up reset.
 * 0x7F selects the register bank. Part 2 goes in 100 ms after part 1. */
static const PMW3901_RegVal_t PMW3901_InitPart1[] = {
    {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00}, {0x7F, 0x05},
    {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14}, {0x5B, 0x32}, {0x5F, 0x34},
    {0x7B, 0x08}, {0x7F, 0x06}, {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F},
    {0x7F, 0x08}, {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
    {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77}, {0x60, 0x78},
    {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50}, {0x7F, 0x0A}, {0x45, 0x60},
    {0x7F, 0x00}, {0x4D, 0x11}, {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F},
    {0x4A, 0x78}, {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
    {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x60}, {0x66, 0x08}, {0x63, 0x78},
    {0x7F, 0x15}, {0x48, 0x58}, {0x7F, 0x07}, {0x41, 0x0D}, {0x43, 0x14},
    {0x4B, 0x0E}, {0x45, 0x0F}, {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10},
    {0x5B, 0x02}, {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
};

static const PMW3901_RegVal_t PMW3901_InitPart2[] = {
    {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06}, {0x62, 0xF0},
    {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0}, {0x6F, 0xD5}, {0x7F, 0x00},
    {0x5B, 0xA0}, {0x4E, 0xA8}, {0x5A, 0x50}, {0x40, 0x80},
};

static void PMW3901_WaitUs(uint32_t us){
    uint32_t t0 = Time_Us();
    while((uint32_t)(Time_Us() - t0) < us){}
}

static void PMW3901_Select(void){
    HAL_GPIO_WritePin(PMW3901_CS_PORT, PMW3901_CS_PIN, GPIO_PIN_RESET);
}

static void PMW3901_Deselect(void){
    HAL_GPIO_WritePin(PMW3901_CS_PORT, PMW3901_CS_PIN, GPIO_PIN_SET);
}

void PMW3901_DMA_Callback(SPI_HandleTypeDef *hspi){
    if(pmw3901DmaSem == NULL) return;

    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(pmw3901DmaSem, &hpw);
    portYIELD_FROM_ISR(hpw);
}

static void PMW3901_ClearDMACompletion(void){
    while(xSemaphoreTake(pmw3901DmaSem, 0) == pdTRUE){}
}

static PMW3901_Status_t PMW3901_Transfer(uint8_t *tx, uint8_t *rx, uint16_t len){
    PMW3901_ClearDMACompletion();
    if(HAL_SPI_TransmitReceive_DMA(&PMW3901_SPI, tx, rx, len) != HAL_OK) return PMW3901_ERROR;
    if(xSemaphoreTake(pmw3901DmaSem, pdMS_TO_TICKS(PMW3901_SPI_TIMEOUT_MS)) != pdTRUE){
        HAL_SPI_Abort(&PMW3901_SPI);
        return PMW3901_ERROR;
    }
    return PMW3901_OK;
}

/* address byte, wait, then len bytes in (MOSI sends 0). The sensor needs the
 * wait between them, so they are two transfers. */
static PMW3901_Status_t PMW3901_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len){
    if(len > PMW3901_BURST_LEN) return PMW3901_ERROR;

    uint8_t addr  = reg & (uint8_t)~PMW3901_SPI_WRITE;
    uint8_t dummy = 0;
    PMW3901_Status_t st;

    PMW3901_Select();
    PMW3901_WaitUs(PMW3901_ADDR_DATA_US);
    st = PMW3901_Transfer(&addr, &dummy, 1);
    if(st == PMW3901_OK){
        PMW3901_WaitUs(PMW3901_ADDR_DATA_US);
        st = PMW3901_Transfer(pmw3901Zeros, buf, len);
    }
    PMW3901_WaitUs(PMW3901_CS_HOLD_US);
    PMW3901_Deselect();
    PMW3901_WaitUs(PMW3901_BETWEEN_US);
    return st;
}

static PMW3901_Status_t PMW3901_ReadReg(uint8_t reg, uint8_t *val){
    return PMW3901_ReadRegs(reg, val, 1);
}

static PMW3901_Status_t PMW3901_WriteReg(uint8_t reg, uint8_t val){
    uint8_t tx[2] = { (uint8_t)(reg | PMW3901_SPI_WRITE), val };
    uint8_t rx[2];
    PMW3901_Status_t st;

    PMW3901_Select();
    PMW3901_WaitUs(PMW3901_ADDR_DATA_US);
    st = PMW3901_Transfer(tx, rx, 2);
    PMW3901_WaitUs(PMW3901_CS_HOLD_US);
    PMW3901_Deselect();
    PMW3901_WaitUs(PMW3901_BETWEEN_US);
    return st;
}

static PMW3901_Status_t PMW3901_WriteTable(const PMW3901_RegVal_t *t, uint16_t n){
    for(uint16_t i = 0; i < n; i++){
        if(PMW3901_WriteReg(t[i].reg, t[i].val) != PMW3901_OK) return PMW3901_ERROR;
    }
    return PMW3901_OK;
}

PMW3901_Status_t PMW3901_Init(void){
    if(pmw3901DmaSem == NULL) pmw3901DmaSem = xSemaphoreCreateBinary();
    if(pmw3901DmaSem == NULL) return PMW3901_ERROR;

    memset(&pmw3901, 0, sizeof(pmw3901));

    /* CS high - low - high resets the sensor's SPI port */
    PMW3901_Deselect();
    vTaskDelay(pdMS_TO_TICKS(2));
    PMW3901_Select();
    vTaskDelay(pdMS_TO_TICKS(2));
    PMW3901_Deselect();
    vTaskDelay(pdMS_TO_TICKS(2));

    if(PMW3901_WriteReg(PMW3901_REG_POWER_UP_RESET, PMW3901_POWER_UP_RESET_VALUE) != PMW3901_OK) return PMW3901_ERROR;
    vTaskDelay(pdMS_TO_TICKS(5));

    uint8_t id = 0, inv = 0;
    if(PMW3901_ReadReg(PMW3901_REG_PRODUCT_ID, &id)          != PMW3901_OK) return PMW3901_ERROR;
    if(PMW3901_ReadReg(PMW3901_REG_INVERSE_PRODUCT_ID, &inv) != PMW3901_OK) return PMW3901_ERROR;
    if(id != PMW3901_PRODUCT_ID_VALUE || inv != PMW3901_INVERSE_ID_VALUE) return PMW3901_INVALID_CHIP;

    /* read the motion registers once, as the datasheet asks after a reset */
    uint8_t dummy;
    for(uint8_t reg = PMW3901_REG_MOTION; reg <= PMW3901_REG_DELTA_Y_H; reg++){
        if(PMW3901_ReadReg(reg, &dummy) != PMW3901_OK) return PMW3901_ERROR;
    }
    vTaskDelay(pdMS_TO_TICKS(1));

    if(PMW3901_WriteTable(PMW3901_InitPart1, sizeof(PMW3901_InitPart1) / sizeof(PMW3901_InitPart1[0])) != PMW3901_OK){
        return PMW3901_ERROR;
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    if(PMW3901_WriteTable(PMW3901_InitPart2, sizeof(PMW3901_InitPart2) / sizeof(PMW3901_InitPart2[0])) != PMW3901_OK){
        return PMW3901_ERROR;
    }

    return PMW3901_Read();
}

PMW3901_Status_t PMW3901_Read(void){
    uint8_t b[PMW3901_BURST_LEN];
    if(PMW3901_ReadRegs(PMW3901_REG_MOTION_BURST, b, PMW3901_BURST_LEN) != PMW3901_OK) return PMW3901_ERROR;

    PMW3901_Burst_t *m = &pmw3901.burst;
    m->motion      = b[0];
    m->observation = b[1];
    m->deltaX      = (int16_t)((b[3] << 8) | b[2]);
    m->deltaY      = (int16_t)((b[5] << 8) | b[4]);
    m->squal       = b[6];
    m->rawDataSum  = b[7];
    m->maxRawData  = b[8];
    m->minRawData  = b[9];
    m->shutter     = (uint16_t)((b[10] << 8) | b[11]);

    uint8_t moved = (m->motion & PMW3901_MOTION_MOT) ? 1 : 0;
    pmw3901.deltaX = moved ? m->deltaX : 0;
    pmw3901.deltaY = moved ? m->deltaY : 0;
    pmw3901.squal  = m->squal;
    pmw3901.valid  = (m->squal >= PMW3901_SQUAL_MIN);
    return PMW3901_OK;
}
