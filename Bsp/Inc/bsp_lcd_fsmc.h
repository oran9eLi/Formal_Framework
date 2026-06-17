/**
 * @file bsp_lcd_fsmc.h
 * @brief Declare the FSMC 8080 bus access layer for the ATK-MD0700 LCD.
 */

#ifndef BSP_LCD_FSMC_H
#define BSP_LCD_FSMC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    BSP_LCD_FSMC_OK = 0,
    BSP_LCD_FSMC_ERROR
} BSP_LcdFsmcResult_t;

typedef struct
{
    uint32_t bank;
    uint32_t address_setup;
    uint32_t address_setup_write;
    uint32_t data_setup_read;
    uint32_t data_setup_write;
    uint8_t bus_width_bits;
} BSP_LcdFsmcConfig_t;

/**
 * @brief Initialize the FSMC bus and LCD backlight GPIO.
 */
BSP_LcdFsmcResult_t BSP_LcdFsmc_Init(
    const BSP_LcdFsmcConfig_t *config);
/**
 * @brief Write one 16-bit LCD command through the FSMC command address.
 */
void BSP_LcdFsmc_WriteCommand(uint16_t command);
/**
 * @brief Write one 16-bit LCD data value through the FSMC data address.
 */
void BSP_LcdFsmc_WriteData(uint16_t data);
/**
 * @brief Read one 16-bit LCD data value through the FSMC data address.
 */
uint16_t BSP_LcdFsmc_ReadData(void);
/**
 * @brief Set the LCD backlight to an on or off state.
 */
void BSP_LcdFsmc_SetBacklight(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif
