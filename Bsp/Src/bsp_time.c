/**
 * @file bsp_time.c
 * @brief 实现设备驱动使用的板级时间接口。
 */

#include "bsp_time.h"

#include "stm32f4xx_hal.h"

/**
 * @brief 返回板级毫秒 tick。
 */
uint32_t BSP_Time_GetTickMs(void)
{
  return HAL_GetTick();
}

/**
 * @brief 延时指定毫秒数。
 */
void BSP_Time_DelayMs(uint32_t delay_ms)
{
  HAL_Delay(delay_ms);
}
