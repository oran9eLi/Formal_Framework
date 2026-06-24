/**
 * @file bsp_lcd_fsmc.h
 * @brief 声明 ATK-MD0700 LCD 的 FSMC 8080 总线访问接口。
 */

#ifndef BSP_LCD_FSMC_H
#define BSP_LCD_FSMC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LCD FSMC 接口返回码。
 */
typedef enum {
  BSP_LCD_FSMC_OK = 0,
  BSP_LCD_FSMC_ERROR
} BSP_LcdFsmcResult_t;

/**
 * @brief LCD FSMC 总线时序配置。
 */
typedef struct {
  uint32_t bank;                /**< FSMC Bank 基地址。 */
  uint32_t address_setup;       /**< 读周期地址建立时间。 */
  uint32_t address_setup_write; /**< 写周期地址建立时间。 */
  uint32_t data_setup_read;     /**< 读周期数据建立时间。 */
  uint32_t data_setup_write;    /**< 写周期数据建立时间。 */
  uint8_t bus_width_bits;       /**< 总线宽度，单位 bit。 */
} BSP_LcdFsmcConfig_t;

/**
 * @brief 初始化 FSMC 总线和 LCD 背光 GPIO。
 */
BSP_LcdFsmcResult_t BSP_LcdFsmc_Init(const BSP_LcdFsmcConfig_t *config);
/**
 * @brief 通过 FSMC 命令地址写入一个 16-bit LCD 命令。
 */
void BSP_LcdFsmc_WriteCommand(uint16_t command);
/**
 * @brief 通过 FSMC 数据地址写入一个 16-bit LCD 数据。
 */
void BSP_LcdFsmc_WriteData(uint16_t data);
/**
 * @brief 通过 FSMC 数据地址读取一个 16-bit LCD 数据。
 */
uint16_t BSP_LcdFsmc_ReadData(void);
/**
 * @brief 设置 LCD 背光开关状态。
 */
void BSP_LcdFsmc_SetBacklight(uint8_t on);

#ifdef __cplusplus
}
#endif

#endif
