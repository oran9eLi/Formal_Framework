/**
 * @file display_ssd1963.h
 * @brief Declare the SSD1963-like LCD controller driver for ATK-MD0700.
 */

#ifndef DISPLAY_SSD1963_H
#define DISPLAY_SSD1963_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_SSD1963_WIDTH  800U
#define DISPLAY_SSD1963_HEIGHT 480U

typedef enum {
  DISPLAY_SSD1963_OK = 0,
  DISPLAY_SSD1963_NOT_READY,
  DISPLAY_SSD1963_ERROR
} Display_Ssd1963Result_t;

/**
 * @brief Initialize the FSMC bus and SSD1963-like controller.
 */
Display_Ssd1963Result_t Display_Ssd1963_Init(void);
/**
 * @brief Read the controller product ID byte used by the old ATK-MD0700 code.
 */
uint8_t Display_Ssd1963_ReadId(void);
/**
 * @brief Draw one RGB565 pixel.
 */
void Display_Ssd1963_DrawPixel(uint16_t x, uint16_t y, uint16_t color);
/**
 * @brief Fill one clipped RGB565 rectangle.
 */
void Display_Ssd1963_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color);
/**
 * @brief Write one clipped RGB565 pixel block.
 */
void Display_Ssd1963_FlushPixels(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels);
/**
 * @brief Fill the full display area with one RGB565 color.
 */
void Display_Ssd1963_Clear(uint16_t color);
/**
 * @brief Return whether the controller has completed initialization.
 */
uint8_t Display_Ssd1963_IsReady(void);
/**
 * @brief Probe whether the controller can still be read through FSMC.
 */
Display_Ssd1963Result_t Display_Ssd1963_Probe(void);

#ifdef __cplusplus
}
#endif

#endif
