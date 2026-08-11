/**
 * @file display_ssd1963.c
 * @brief Implement the SSD1963-like LCD controller driver for ATK-MD0700.
 */

#include "display_ssd1963.h"

#include "bsp_config.h"
#include "bsp_lcd_fsmc.h"
#include "stm32f4xx_hal.h"

#define DISPLAY_SSD1963_POWER_ON_MS    120U
#define DISPLAY_SSD1963_PLL_LOCK_MS    100U

static uint8_t s_ssd1963_ready;

/**
 * @brief Write one LCD controller command.
 */
static void Display_Ssd1963_WriteCommand(uint16_t command)
{
  BSP_LcdFsmc_WriteCommand(command);
}

/**
 * @brief Write one LCD controller data word.
 */
static void Display_Ssd1963_WriteData(uint16_t data)
{
  BSP_LcdFsmc_WriteData(data);
}

/**
 * @brief Set the active GRAM write window and enter memory-write mode.
 */
static void Display_Ssd1963_SetWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
  Display_Ssd1963_WriteCommand(0x2AU);
  Display_Ssd1963_WriteData((uint16_t)(x0 >> 8));
  Display_Ssd1963_WriteData((uint16_t)(x0 & 0x00FFU));
  Display_Ssd1963_WriteData((uint16_t)(x1 >> 8));
  Display_Ssd1963_WriteData((uint16_t)(x1 & 0x00FFU));

  Display_Ssd1963_WriteCommand(0x2BU);
  Display_Ssd1963_WriteData((uint16_t)(y0 >> 8));
  Display_Ssd1963_WriteData((uint16_t)(y0 & 0x00FFU));
  Display_Ssd1963_WriteData((uint16_t)(y1 >> 8));
  Display_Ssd1963_WriteData((uint16_t)(y1 & 0x00FFU));

  Display_Ssd1963_WriteCommand(0x2CU);
}

/**
 * @brief Send the SSD1963 startup and ATK-MD0700 timing sequence.
 */
static void Display_Ssd1963_InitController(void)
{
  Display_Ssd1963_WriteCommand(0xE2U);
  Display_Ssd1963_WriteData(0x1DU);
  Display_Ssd1963_WriteData(0x02U);
  Display_Ssd1963_WriteData(0x04U);
  HAL_Delay(1U);

  Display_Ssd1963_WriteCommand(0xE0U);
  Display_Ssd1963_WriteData(0x01U);
  HAL_Delay(DISPLAY_SSD1963_PLL_LOCK_MS);

  Display_Ssd1963_WriteCommand(0xE0U);
  Display_Ssd1963_WriteData(0x03U);
  HAL_Delay(12U);

  Display_Ssd1963_WriteCommand(0x01U);
  HAL_Delay(10U);

  Display_Ssd1963_WriteCommand(0xE6U);
  Display_Ssd1963_WriteData(0x2FU);
  Display_Ssd1963_WriteData(0xFFU);
  Display_Ssd1963_WriteData(0xFFU);

  Display_Ssd1963_WriteCommand(0xB0U);
  Display_Ssd1963_WriteData(0x20U);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x03U);
  Display_Ssd1963_WriteData(0x1FU);
  Display_Ssd1963_WriteData(0x01U);
  Display_Ssd1963_WriteData(0xDFU);
  Display_Ssd1963_WriteData(0x00U);

  Display_Ssd1963_WriteCommand(0xB4U);
  Display_Ssd1963_WriteData(0x04U);
  Display_Ssd1963_WriteData(0x1FU);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x2EU);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);

  Display_Ssd1963_WriteCommand(0xB6U);
  Display_Ssd1963_WriteData(0x02U);
  Display_Ssd1963_WriteData(0x0CU);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x17U);
  Display_Ssd1963_WriteData(0x16U);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);

  Display_Ssd1963_WriteCommand(0xF0U);
  Display_Ssd1963_WriteData(0x03U);

  Display_Ssd1963_WriteCommand(0x29U);
  Display_Ssd1963_WriteCommand(0xD0U);
  Display_Ssd1963_WriteData(0x00U);

  Display_Ssd1963_WriteCommand(0xBEU); /* SET_PWM_CONF：背光 PWM */
  Display_Ssd1963_WriteData(0x05U);    /* PWM 频率 */
  Display_Ssd1963_WriteData(0xFFU);    /* PWM 占空比=255，背光最高亮度 */
  Display_Ssd1963_WriteData(0x01U);    /* PWM 使能、由主机控制 */
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);
  Display_Ssd1963_WriteData(0x00U);

  Display_Ssd1963_WriteCommand(0xB8U);
  Display_Ssd1963_WriteData(0x03U);
  Display_Ssd1963_WriteData(0x01U);

  Display_Ssd1963_WriteCommand(0xBAU);
  Display_Ssd1963_WriteData(0x01U);

  Display_Ssd1963_WriteCommand(0x36U);
  Display_Ssd1963_WriteData(0x00U);
}

/**
 * @brief Initialize the FSMC bus and SSD1963-like controller.
 */
Display_Ssd1963Result_t Display_Ssd1963_Init(void)
{
  BSP_LcdFsmcConfig_t config;

  /*
   * On a cold power-up the LCD module's supply rail and the SSD1963 internal
   * power-on reset need time to settle before the bus may be driven.  Wait for
   * that settle window before touching the FSMC so a cold boot behaves like the
   * (already-stable) warm-reset case.
   */
  HAL_Delay(DISPLAY_SSD1963_POWER_ON_MS);

  config.bank                = 4U;
  config.address_setup       = BSP_LCD_FSMC_ADDRESS_SETUP;
  config.address_setup_write = BSP_LCD_FSMC_ADDRESS_SETUP_WRITE;
  config.data_setup_read     = BSP_LCD_FSMC_DATA_SETUP_READ;
  config.data_setup_write    = BSP_LCD_FSMC_DATA_SETUP_WRITE;
  config.bus_width_bits      = BSP_DISPLAY_BUS_WIDTH_BITS;

  if (BSP_LcdFsmc_Init(&config) != BSP_LCD_FSMC_OK) {
    s_ssd1963_ready = 0U;
    return DISPLAY_SSD1963_ERROR;
  }

  /*
   * The SSD1963 8080 interface is write-driven: a correct init sequence lights
   * the panel without any register read-back.  The FSMC read used to fetch the
   * product ID is the most state/timing-sensitive access and is unreliable on a
   * cold boot, so it must NOT gate whether the driver may draw.  Run the full
   * controller sequence and treat the panel as ready once it completes; the ID
   * read is kept only as a best-effort diagnostic.
   */
  Display_Ssd1963_InitController();
  (void)Display_Ssd1963_ReadId();

  s_ssd1963_ready = 1U;
  Display_Ssd1963_Clear(0xFFFFU);
  BSP_LcdFsmc_SetBacklight(1U);

  return DISPLAY_SSD1963_OK;
}

/**
 * @brief Read the controller product ID byte used by the old ATK-MD0700 code.
 */
uint8_t Display_Ssd1963_ReadId(void)
{
  uint8_t pid;

  Display_Ssd1963_WriteCommand(0xA1U);
  (void)BSP_LcdFsmc_ReadData();
  (void)BSP_LcdFsmc_ReadData();
  pid = (uint8_t)BSP_LcdFsmc_ReadData();

  return pid;
}

/**
 * @brief Draw one RGB565 pixel.
 */
void Display_Ssd1963_DrawPixel(uint16_t x, uint16_t y, uint16_t color)
{
  Display_Ssd1963_FillRect(x, y, 1U, 1U, color);
}

/**
 * @brief Fill one clipped RGB565 rectangle.
 */
void Display_Ssd1963_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t color)
{
  uint16_t clipped_width;
  uint16_t clipped_height;
  uint32_t pixel_count;

  if ((s_ssd1963_ready == 0U) || (width == 0U) || (height == 0U) || (x >= DISPLAY_SSD1963_WIDTH) || (y >= DISPLAY_SSD1963_HEIGHT)) { return; }

  clipped_width  = width;
  clipped_height = height;
  if (((uint32_t)x + clipped_width) > DISPLAY_SSD1963_WIDTH) { clipped_width = (uint16_t)(DISPLAY_SSD1963_WIDTH - x); }
  if (((uint32_t)y + clipped_height) > DISPLAY_SSD1963_HEIGHT) { clipped_height = (uint16_t)(DISPLAY_SSD1963_HEIGHT - y); }

  Display_Ssd1963_SetWindow(x, y, (uint16_t)(x + clipped_width - 1U), (uint16_t)(y + clipped_height - 1U));
  pixel_count = (uint32_t)clipped_width * clipped_height;
  while (pixel_count > 0U) {
    Display_Ssd1963_WriteData(color);
    pixel_count--;
  }
}

/**
 * @brief Write one clipped RGB565 pixel block to GRAM.
 */
void Display_Ssd1963_FlushPixels(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const uint16_t *pixels)
{
  uint16_t clipped_width;
  uint16_t clipped_height;
  uint16_t row;
  uint16_t col;
  uint16_t skip;
  const uint16_t *row_pixels;

  if ((s_ssd1963_ready == 0U) || (pixels == 0) || (width == 0U) || (height == 0U) || (x >= DISPLAY_SSD1963_WIDTH) || (y >= DISPLAY_SSD1963_HEIGHT)) { return; }

  clipped_width  = width;
  clipped_height = height;
  if (((uint32_t)x + clipped_width) > DISPLAY_SSD1963_WIDTH) { clipped_width = (uint16_t)(DISPLAY_SSD1963_WIDTH - x); }
  if (((uint32_t)y + clipped_height) > DISPLAY_SSD1963_HEIGHT) { clipped_height = (uint16_t)(DISPLAY_SSD1963_HEIGHT - y); }

  Display_Ssd1963_SetWindow(x, y, (uint16_t)(x + clipped_width - 1U), (uint16_t)(y + clipped_height - 1U));
  skip = (uint16_t)(width - clipped_width);
  row_pixels = pixels;
  for (row = 0U; row < clipped_height; row++) {
    for (col = 0U; col < clipped_width; col++) {
      Display_Ssd1963_WriteData(*row_pixels);
      row_pixels++;
    }
    row_pixels += skip;
  }
}

/**
 * @brief Fill the full display area with one RGB565 color.
 */
void Display_Ssd1963_Clear(uint16_t color)
{
  Display_Ssd1963_FillRect(0U, 0U, DISPLAY_SSD1963_WIDTH, DISPLAY_SSD1963_HEIGHT, color);
}

/**
 * @brief Return whether the controller has completed initialization.
 */
uint8_t Display_Ssd1963_IsReady(void)
{
  return s_ssd1963_ready;
}
