/**
 * @file bsp_lcd_fsmc.h
 * @brief 声明 ATK-MD0700 LCD 的 FSMC 8080 总线访问接口。
 * @details 属于 BSP 层，仅提供原始命令、数据与同步批量写入；不得解释像素窗口或执行页面逻辑。
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
 * @brief 同步写入连续的 16-bit 原始数据，不保存调用方指针。
 * @param[in] data 源数据，count 非零时不得为空。
 * @param[in] count 数据数量，单位为 16-bit word；0 表示不写入。
 * @details 仅 LCD 所属任务调用；不修改总线时序，不屏蔽中断，不启动 DMA。
 */
void BSP_LcdFsmc_WriteDataBuffer(const uint16_t *data, uint32_t count);
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
