#ifndef INC_DRIVER_ILI9341_H_
#define INC_DRIVER_ILI9341_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

#define ILI9341_CS_PORT     GPIOB
#define ILI9341_CS_PIN      GPIO_PIN_6
#define ILI9341_DC_PORT     GPIOB
#define ILI9341_DC_PIN      GPIO_PIN_7
#define ILI9341_RST_PORT    GPIOB
#define ILI9341_RST_PIN     GPIO_PIN_8

#define ILI9341_WIDTH       320     /* landscape */
#define ILI9341_HEIGHT      240

/* 	0x28	Landscape (current)
	0xE8	Landscape, turned 180°
	0x48	Portrait
	0x88	Portrait, turned 180° */
#define ILI9341_MADCTL      0x88

#define RGB565(r, g, b)     ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
#define C_BLACK             0x0000
#define C_WHITE             0xFFFF
#define C_GREY              RGB565(128, 128, 128)
#define C_GREEN             RGB565(0, 220, 0)
#define C_YELLOW            RGB565(255, 220, 0)
#define C_RED               RGB565(255, 0, 0)
#define C_CYAN              RGB565(0, 220, 255)
#define C_NAVY              RGB565(0, 0, 120)

void ILI9341_Init(void);
void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9341_DrawText(uint16_t x, uint16_t y, const char *s, uint16_t n, uint16_t fg, uint16_t bg);

#endif /* INC_DRIVER_ILI9341_H_ */
