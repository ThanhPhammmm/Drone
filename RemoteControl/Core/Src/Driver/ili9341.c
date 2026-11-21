#include "ili9341.h"
#include "font8x16.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

extern SPI_HandleTypeDef hspi3;
#define ILI_SPI				(&hspi3)

#define ILI_CMD_TIMEOUT_MS	10U
#define ILI_DMA_TIMEOUT_MS	200U	/* 5 KB at 6.25 Mbit/s is ~7 ms */
#define ILI_TEXT_CHUNK		20U	/* characters per SPI window */
#define ILI_BUF_BYTES		(ILI_TEXT_CHUNK * FONT_W * FONT_H * 2U)	/* 5120 bytes */
#define ILI_DMA_MIN_BYTES	32U	/* shorter blocks go out by polling */

#define CS_LOW()	HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_RESET)
#define CS_HIGH()	HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET)
#define DC_CMD()	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_RESET)
#define DC_DATA()	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET)

static SemaphoreHandle_t dmaDone;
static uint8_t txBuf[ILI_BUF_BYTES];	/* pixels for one transfer, RGB565 big-endian */

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi){
	if(hspi->Instance == SPI3 && dmaDone != NULL){
		BaseType_t woken = pdFALSE;
		xSemaphoreGiveFromISR(dmaDone, &woken);
		portYIELD_FROM_ISR(woken);
	}
}

static void WriteCmd(uint8_t cmd){
	DC_CMD();
	HAL_SPI_Transmit(ILI_SPI, &cmd, 1, ILI_CMD_TIMEOUT_MS);
}

static void WriteData(const uint8_t *data, uint16_t len){
	DC_DATA();
	HAL_SPI_Transmit(ILI_SPI, (uint8_t *)data, len, ILI_CMD_TIMEOUT_MS);
}

/* Pixel data: by DMA, the task sleeps until it is done. */
static void WritePixels(const uint8_t *data, uint16_t len){
	DC_DATA();
	if(len < ILI_DMA_MIN_BYTES){
		HAL_SPI_Transmit(ILI_SPI, (uint8_t *)data, len, ILI_CMD_TIMEOUT_MS);
		return;
	}
	if(HAL_SPI_Transmit_DMA(ILI_SPI, (uint8_t *)data, len) != HAL_OK){
		return;
	}
	if(xSemaphoreTake(dmaDone, pdMS_TO_TICKS(ILI_DMA_TIMEOUT_MS)) != pdTRUE){
		HAL_SPI_Abort(ILI_SPI);	/* never leave the bus stuck: next transfer starts clean */
	}
}

static void SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1){
	uint8_t d[4];
	WriteCmd(0x2A);	/* column address set */
	d[0] = x0 >> 8; d[1] = x0 & 0xFF; d[2] = x1 >> 8; d[3] = x1 & 0xFF;
	WriteData(d, 4);
	WriteCmd(0x2B);	/* page address set */
	d[0] = y0 >> 8; d[1] = y0 & 0xFF; d[2] = y1 >> 8; d[3] = y1 & 0xFF;
	WriteData(d, 4);
	WriteCmd(0x2C);	/* memory write */
}

/* Init table: command, number of data bytes, data. ILI_DELAY in the count = wait after. */
#define ILI_DELAY	0x80
static const uint8_t initCmds[] = {
	0xEF, 3, 0x03, 0x80, 0x02,
	0xCF, 3, 0x00, 0xC1, 0x30,								/* power control B */
	0xED, 4, 0x64, 0x03, 0x12, 0x81,						/* power on sequence */
	0xE8, 3, 0x85, 0x00, 0x78,								/* driver timing A */
	0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,					/* power control A */
	0xF7, 1, 0x20,											/* pump ratio */
	0xEA, 2, 0x00, 0x00,									/* driver timing B */
	0xC0, 1, 0x23,											/* power control 1 */
	0xC1, 1, 0x10,											/* power control 2 */
	0xC5, 2, 0x3E, 0x28,									/* VCOM control 1 */
	0xC7, 1, 0x86,											/* VCOM control 2 */
	0x36, 1, ILI9341_MADCTL,								/* orientation */
	0x3A, 1, 0x55,											/* 16 bits per pixel */
	0xB1, 2, 0x00, 0x18,									/* frame rate 79 Hz */
	0xB6, 3, 0x08, 0x82, 0x27,								/* display function control */
	0xF2, 1, 0x00,											/* 3-gamma off */
	0x26, 1, 0x01,											/* gamma curve 1 */
	0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1,
			  0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,		/* positive gamma */
	0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1,
			  0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,		/* negative gamma */
	0x11, ILI_DELAY,										/* sleep out, then 120 ms */
	0x29, ILI_DELAY,										/* display on */
	0x00													/* end */
};

void ILI9341_Init(void){
	if(dmaDone == NULL) dmaDone = xSemaphoreCreateBinary();

	__HAL_RCC_GPIOB_CLK_ENABLE();
	HAL_GPIO_WritePin(ILI9341_CS_PORT, ILI9341_CS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ILI9341_DC_PORT, ILI9341_DC_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
	GPIO_InitTypeDef g = {0};
	g.Mode = GPIO_MODE_OUTPUT_PP;
	g.Pull = GPIO_NOPULL;
	g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	g.Pin = ILI9341_CS_PIN; HAL_GPIO_Init(ILI9341_CS_PORT, &g);
	g.Pin = ILI9341_DC_PIN; HAL_GPIO_Init(ILI9341_DC_PORT, &g);
	g.Pin = ILI9341_RST_PIN; HAL_GPIO_Init(ILI9341_RST_PORT, &g);

	/* hardware reset */
	vTaskDelay(pdMS_TO_TICKS(5));
	HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_RESET);
	vTaskDelay(pdMS_TO_TICKS(20));
	HAL_GPIO_WritePin(ILI9341_RST_PORT, ILI9341_RST_PIN, GPIO_PIN_SET);
	vTaskDelay(pdMS_TO_TICKS(150));

	CS_LOW();
	WriteCmd(0x01);	/* software reset */
	CS_HIGH();
	vTaskDelay(pdMS_TO_TICKS(150));

	const uint8_t *p = initCmds;
	while(*p != 0x00){
		uint8_t cmd = *p++;
		uint8_t n = *p++;
		CS_LOW();
		WriteCmd(cmd);
		if(n & ILI_DELAY){
			CS_HIGH();
			vTaskDelay(pdMS_TO_TICKS(120));
			continue;
		}
		if(n) WriteData(p, n);
		CS_HIGH();
		p += n;
	}

	ILI9341_FillRect(0, 0, ILI9341_WIDTH, ILI9341_HEIGHT, C_BLACK);
}

void ILI9341_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color){
	if(x >= ILI9341_WIDTH || y >= ILI9341_HEIGHT || w == 0 || h == 0) return;
	if(x + w > ILI9341_WIDTH) w = ILI9341_WIDTH - x;
	if(y + h > ILI9341_HEIGHT) h = ILI9341_HEIGHT - y;

	uint32_t total = (uint32_t)w * h;
	uint32_t chunk = (total < ILI_BUF_BYTES / 2U) ? total : ILI_BUF_BYTES / 2U;
	for(uint32_t i = 0; i < chunk; i++){
		txBuf[2U * i] = color >> 8;
		txBuf[2U * i + 1U] = color & 0xFF;
	}

	CS_LOW();
	SetWindow(x, y, x + w - 1U, y + h - 1U);
	while(total > 0U){
		uint32_t n = (total < chunk) ? total : chunk;
		WritePixels(txBuf, (uint16_t)(n * 2U));
		total -= n;
	}
	CS_HIGH();
}

void ILI9341_DrawText(uint16_t x, uint16_t y, const char *s, uint16_t n, uint16_t fg, uint16_t bg){
	const uint8_t fh = fg >> 8, fl = fg & 0xFF, bh = bg >> 8, bl = bg & 0xFF;

	while(n > 0U){
		uint16_t k = (n < ILI_TEXT_CHUNK) ? n : ILI_TEXT_CHUNK;
		if(x + k * FONT_W > ILI9341_WIDTH || y + FONT_H > ILI9341_HEIGHT) return;

		/* the window is k characters wide: pixel rows run across all k glyphs */
		uint8_t *p = txBuf;
		for(uint8_t row = 0; row < FONT_H; row++){
			for(uint16_t i = 0; i < k; i++){
				char c = s[i];
				if(c < FONT_FIRST || c > FONT_LAST) c = '?';
				uint8_t bits = font8x16[c - FONT_FIRST][row];
				for(uint8_t b = 0; b < FONT_W; b++){
					if(bits & (0x80U >> b)){ *p++ = fh; *p++ = fl; }
					else { *p++ = bh; *p++ = bl; }
				}
			}
		}

		CS_LOW();
		SetWindow(x, y, x + k * FONT_W - 1U, y + FONT_H - 1U);
		WritePixels(txBuf, (uint16_t)(p - txBuf));
		CS_HIGH();

		x += k * FONT_W;
		s += k;
		n -= k;
	}
}
