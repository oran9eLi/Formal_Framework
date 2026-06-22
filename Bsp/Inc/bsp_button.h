/**
 * @file bsp_button.h
 * @brief 声明板载按键 GPIO 输入接口。
 *
 * @details
 * BSP 层只返回极性归一化后的瞬时按下状态，不做去抖、边沿识别或业务动作。
 */

#ifndef BSP_BUTTON_H
#define BSP_BUTTON_H

#include <stdint.h>
#include "bsp_status.h"

/**
 * @brief 板载按键逻辑编号。
 */
typedef enum {
  BSP_BUTTON_WKUP = 0, /**< WKUP 按键，PA0，按下为高电平。 */
  BSP_BUTTON_KEY0,     /**< KEY0 按键，PE4，按下为低电平。 */
  BSP_BUTTON_KEY1,     /**< KEY1 按键，PE3，按下为低电平。 */
  BSP_BUTTON_KEY2,     /**< KEY2 按键，PE2，按下为低电平。 */
  BSP_BUTTON_COUNT     /**< 按键数量，必须保持为最后一项。 */
} BSP_ButtonId_t;

/**
 * @brief 初始化所有板载按键 GPIO。
 *
 * @return 初始化结果。
 */
BSP_Status_t BSP_Button_Init(void);

/**
 * @brief 读取按键是否处于按下状态。
 *
 * @param[in] button 按键编号。
 *
 * @return 1 表示按下，0 表示未按下或编号非法。
 */
uint8_t BSP_Button_IsPressed(BSP_ButtonId_t button);

#endif
