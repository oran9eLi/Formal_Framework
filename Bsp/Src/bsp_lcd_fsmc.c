/**
 * @file bsp_lcd_fsmc.c
 * @brief 实现 ATK-MD0700 LCD 的 FSMC 8080 总线访问接口。
 */

#include "bsp_lcd_fsmc.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

#define BSP_LCD_FSMC_CMD_REG    (*(volatile uint16_t *)BSP_LCD_FSMC_CMD_ADDR)
#define BSP_LCD_FSMC_DAT_REG    (*(volatile uint16_t *)BSP_LCD_FSMC_DAT_ADDR)
#define BSP_LCD_FSMC_BCR_VALUE  (FSMC_BCR1_WREN | FSMC_BCR1_MWID_0 | FSMC_BCR1_EXTMOD)
#define BSP_LCD_FSMC_ADDSET_POS 0U
#define BSP_LCD_FSMC_DATAST_POS 8U

void HAL_DisplayLcdMspInit(void);

/**
 * @brief 根据 LCD 总线配置写入 FSMC 时序寄存器。
 */
static void BSP_LcdFsmc_ApplyTiming(const BSP_LcdFsmcConfig_t *config)
{
  uint32_t read_timing;
  uint32_t write_timing;

  read_timing  = ((config->address_setup & 0x0FU) << BSP_LCD_FSMC_ADDSET_POS) | ((config->data_setup_read & 0xFFU) << BSP_LCD_FSMC_DATAST_POS);
  write_timing = ((config->address_setup_write & 0x0FU) << BSP_LCD_FSMC_ADDSET_POS) | ((config->data_setup_write & 0xFFU) << BSP_LCD_FSMC_DATAST_POS);

  FSMC_Bank1->BTCR[BSP_LCD_FSMC_BCR_INDEX]   = BSP_LCD_FSMC_BCR_VALUE;
  FSMC_Bank1->BTCR[BSP_LCD_FSMC_BTR_INDEX]   = read_timing;
  FSMC_Bank1E->BWTR[BSP_LCD_FSMC_BWTR_INDEX] = write_timing;
  FSMC_Bank1->BTCR[BSP_LCD_FSMC_BCR_INDEX]   = BSP_LCD_FSMC_BCR_VALUE | FSMC_BCR1_MBKEN;
}

/**
 * @brief 初始化 FSMC 总线和 LCD 背光 GPIO。
 */
BSP_LcdFsmcResult_t BSP_LcdFsmc_Init(const BSP_LcdFsmcConfig_t *config)
{
  volatile uint32_t delay;

  if ((config == 0) || (config->bus_width_bits != BSP_DISPLAY_BUS_WIDTH_BITS)) { return BSP_LCD_FSMC_ERROR; }

  HAL_DisplayLcdMspInit();
  __HAL_RCC_FSMC_CLK_ENABLE();
  delay = READ_BIT(RCC->AHB3ENR, RCC_AHB3ENR_FSMCEN);
  (void)delay;

  BSP_LcdFsmc_ApplyTiming(config);
  HAL_Delay(5U);
  BSP_LcdFsmc_SetBacklight(0U);

  return BSP_LCD_FSMC_OK;
}

/**
 * @brief 通过 FSMC 命令地址写入一个 16-bit LCD 命令。
 */
void BSP_LcdFsmc_WriteCommand(uint16_t command)
{
  BSP_LCD_FSMC_CMD_REG = command;
}

/**
 * @brief 通过 FSMC 数据地址写入一个 16-bit LCD 数据。
 */
void BSP_LcdFsmc_WriteData(uint16_t data)
{
  BSP_LCD_FSMC_DAT_REG = data;
}

/**
 * @brief 通过 FSMC 数据地址读取一个 16-bit LCD 数据。
 */
uint16_t BSP_LcdFsmc_ReadData(void)
{
  uint16_t data;

  __NOP();
  __NOP();
  data = BSP_LCD_FSMC_DAT_REG;

  return data;
}

/**
 * @brief 设置 LCD 背光开关状态。
 */
void BSP_LcdFsmc_SetBacklight(uint8_t on)
{
  GPIO_PinState inactive_level;

  inactive_level = (BSP_LCD_BL_ACTIVE_LEVEL == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
  HAL_GPIO_WritePin(BSP_LCD_BL_PORT, BSP_LCD_BL_PIN, on ? BSP_LCD_BL_ACTIVE_LEVEL : inactive_level);
}
