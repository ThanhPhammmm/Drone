#include "bmp388.h"
#include "bmp388_reg.h"
#include "i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

#define BARO_I2C                    hi2c2
#define BMP388_I2C_TIMEOUT_MS       10
#define BMP388_GROUND_REF_SAMPLES   100     /* ~1 s at 100 Hz */

BMP388_Handle_t bmp388;

static BMP388_Status_t BMP388_WriteReg(uint8_t reg, uint8_t val){
    if(HAL_I2C_Mem_Write(&BARO_I2C, bmp388.addr, reg, I2C_MEMADD_SIZE_8BIT,
                          &val, 1, BMP388_I2C_TIMEOUT_MS) != HAL_OK){
        return BMP388_ERROR;
    }
    return BMP388_OK;
}

static BMP388_Status_t BMP388_ReadRegs(uint8_t reg, uint8_t *buf, uint16_t len){
    if(HAL_I2C_Mem_Read(&BARO_I2C, bmp388.addr, reg, I2C_MEMADD_SIZE_8BIT,
                         buf, len, BMP388_I2C_TIMEOUT_MS) != HAL_OK){
        return BMP388_ERROR;
    }
    return BMP388_OK;
}

static BMP388_Status_t BMP388_ProbeAddress(void){
    const uint8_t candidates[2] = { BMP388_I2C_ADDR_PRIMARY, BMP388_I2C_ADDR_SECONDARY };
    uint8_t chipId = 0;

    for(uint8_t i = 0; i < 2; i++){
        bmp388.addr = candidates[i];
        if(BMP388_ReadRegs(BMP388_REG_CHIP_ID, &chipId, 1) != BMP388_OK) continue;
        if(chipId == BMP388_CHIP_ID_VALUE || chipId == BMP390_CHIP_ID_VALUE){
            return BMP388_OK;
        }
    }
    return BMP388_INVALID_CHIP;
}

static BMP388_Status_t BMP388_ReadCalib(void){
    uint8_t b[BMP388_CALIB_LEN];
    if(BMP388_ReadRegs(BMP388_REG_CALIB_DATA, b, BMP388_CALIB_LEN) != BMP388_OK){
        return BMP388_ERROR;
    }

    uint16_t nvm_t1  = (uint16_t)((b[1]  << 8) | b[0]);
    uint16_t nvm_t2  = (uint16_t)((b[3]  << 8) | b[2]);
    int8_t   nvm_t3  = (int8_t)b[4];
    int16_t  nvm_p1  = (int16_t) ((b[6]  << 8) | b[5]);
    int16_t  nvm_p2  = (int16_t) ((b[8]  << 8) | b[7]);
    int8_t   nvm_p3  = (int8_t)b[9];
    int8_t   nvm_p4  = (int8_t)b[10];
    uint16_t nvm_p5  = (uint16_t)((b[12] << 8) | b[11]);
    uint16_t nvm_p6  = (uint16_t)((b[14] << 8) | b[13]);
    int8_t   nvm_p7  = (int8_t)b[15];
    int8_t   nvm_p8  = (int8_t)b[16];
    int16_t  nvm_p9  = (int16_t) ((b[18] << 8) | b[17]);
    int8_t   nvm_p10 = (int8_t)b[19];
    int8_t   nvm_p11 = (int8_t)b[20];

    bmp388.calib.t1  = (float)nvm_t1 * 256.0f;                    /* / 2^-8  */
    bmp388.calib.t2  = (float)nvm_t2 / 1073741824.0f;             /* / 2^30  */
    bmp388.calib.t3  = (float)nvm_t3 / 281474976710656.0f;        /* / 2^48  */

    bmp388.calib.p1  = ((float)nvm_p1 - 16384.0f) / 1048576.0f;   /* (x-2^14)/2^20 */
    bmp388.calib.p2  = ((float)nvm_p2 - 16384.0f) / 536870912.0f; /* (x-2^14)/2^29 */
    bmp388.calib.p3  = (float)nvm_p3  / 4294967296.0f;            /* / 2^32  */
    bmp388.calib.p4  = (float)nvm_p4  / 137438953472.0f;          /* / 2^37  */
    bmp388.calib.p5  = (float)nvm_p5  * 8.0f;                     /* / 2^-3  */
    bmp388.calib.p6  = (float)nvm_p6  / 64.0f;                    /* / 2^6   */
    bmp388.calib.p7  = (float)nvm_p7  / 256.0f;                   /* / 2^8   */
    bmp388.calib.p8  = (float)nvm_p8  / 32768.0f;                 /* / 2^15  */
    bmp388.calib.p9  = (float)nvm_p9  / 281474976710656.0f;       /* / 2^48  */
    bmp388.calib.p10 = (float)nvm_p10 / 281474976710656.0f;       /* / 2^48  */
    bmp388.calib.p11 = (float)nvm_p11 / 36893488147419103232.0f;  /* / 2^65  */

    bmp388.calib.t_lin = 0.0f;
    return BMP388_OK;
}

BMP388_Status_t BMP388_Init(void){
    if(BMP388_ProbeAddress() != BMP388_OK) return BMP388_INVALID_CHIP;

    if(BMP388_WriteReg(BMP388_REG_CMD, BMP388_CMD_SOFT_RESET) != BMP388_OK) return BMP388_ERROR;
    vTaskDelay(pdMS_TO_TICKS(10));

    if(BMP388_ReadCalib() != BMP388_OK) return BMP388_ERROR;

    if(BMP388_WriteReg(BMP388_REG_OSR, BMP388_OSR_P_8 | BMP388_OSR_T_1) != BMP388_OK) return BMP388_ERROR;
    if(BMP388_WriteReg(BMP388_REG_ODR, BMP388_ODR_50HZ) != BMP388_OK) return BMP388_ERROR;
    if(BMP388_WriteReg(BMP388_REG_CONFIG, BMP388_IIR_COEF_3) != BMP388_OK) return BMP388_ERROR;

    uint8_t pwr = BMP388_PWR_PRESS_EN | BMP388_PWR_TEMP_EN | BMP388_PWR_MODE_NORMAL;
    if(BMP388_WriteReg(BMP388_REG_PWR_CTRL, pwr) != BMP388_OK) return BMP388_ERROR;

    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t err = 0;
    if(BMP388_ReadRegs(BMP388_REG_ERR, &err, 1) != BMP388_OK) return BMP388_ERROR;
    if(err & (BMP388_ERR_FATAL | BMP388_ERR_CONF)) return BMP388_CONFIG_ERROR;

    BMP388_GroundRefReset();
    return BMP388_OK;
}

static void BMP388_CompensateTemperature(void){
    float d1 = (float)bmp388.raw.temperature - bmp388.calib.t1;
    float d2 = d1 * bmp388.calib.t2;

    bmp388.calib.t_lin   = d2 + (d1 * d1) * bmp388.calib.t3;
    bmp388.temperature_c = bmp388.calib.t_lin;
}

static void BMP388_CompensatePressure(void){
    const float t  = bmp388.calib.t_lin;
    const float t2 = t * t;
    const float t3 = t2 * t;
    const float p  = (float)bmp388.raw.pressure;
    const float p2 = p * p;
    const float p3 = p2 * p;

    float out1 = bmp388.calib.p5
               + bmp388.calib.p6 * t
               + bmp388.calib.p7 * t2
               + bmp388.calib.p8 * t3;

    float out2 = p * (bmp388.calib.p1
                    + bmp388.calib.p2 * t
                    + bmp388.calib.p3 * t2
                    + bmp388.calib.p4 * t3);

    float out3 = p2 * (bmp388.calib.p9 + bmp388.calib.p10 * t)
               + p3 *  bmp388.calib.p11;

    bmp388.pressure_pa = out1 + out2 + out3;
}

uint8_t BMP388_DataReady(void){
    uint8_t status = 0;
    if(BMP388_ReadRegs(BMP388_REG_STATUS, &status, 1) != BMP388_OK) return 0;
    return (status & BMP388_STATUS_DRDY_PRESS) ? 1 : 0;
}

BMP388_Status_t BMP388_Read(void){
    uint8_t buf[6];
    if(BMP388_ReadRegs(BMP388_REG_DATA_0, buf, 6) != BMP388_OK) return BMP388_ERROR;

    bmp388.raw.pressure    = ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | buf[0];
    bmp388.raw.temperature = ((uint32_t)buf[5] << 16) | ((uint32_t)buf[4] << 8) | buf[3];

    BMP388_CompensateTemperature();
    BMP388_CompensatePressure();

    if(bmp388.groundRef.valid){
        bmp388.altitude_m = 44330.0f *
            (1.0f - powf(bmp388.pressure_pa / bmp388.groundRef.pressure_pa, 0.1902949f));
    }
    else{
        bmp388.altitude_m = 0.0f;
    }

    return BMP388_OK;
}

void BMP388_GroundRefReset(void){
    bmp388.groundRef.pressureSum = 0.0f;
    bmp388.groundRef.samples     = 0;
    bmp388.groundRef.pressure_pa = 0.0f;
    bmp388.groundRef.valid       = 0;
}

void BMP388_GroundRefAccumulate(void){
    bmp388.groundRef.pressureSum += bmp388.pressure_pa;
    bmp388.groundRef.samples++;

    if(bmp388.groundRef.samples >= BMP388_GROUND_REF_SAMPLES){
        bmp388.groundRef.pressure_pa = bmp388.groundRef.pressureSum /
                                       (float)bmp388.groundRef.samples;
        bmp388.groundRef.valid = 1;
    }
}
