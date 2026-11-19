#ifndef INC_DRIVER_FONT8X16_H_
#define INC_DRIVER_FONT8X16_H_

#include <stdint.h>

#define FONT_W      8
#define FONT_H      16
#define FONT_FIRST  32      /* ' ' */
#define FONT_LAST   126     /* '~' */

/* One byte per pixel row, MSB = leftmost pixel. */
extern const uint8_t font8x16[FONT_LAST - FONT_FIRST + 1][FONT_H];

#endif /* INC_DRIVER_FONT8X16_H_ */
