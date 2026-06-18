/**
 * @file bsp_time.c
 * @brief Implement board time helpers.
 */

#include "bsp_time.h"

#include "stm32f4xx_hal.h"

uint32_t BSP_Time_GetTickMs(void)
{
    return HAL_GetTick();
}

void BSP_Time_DelayMs(uint32_t delay_ms)
{
    HAL_Delay(delay_ms);
}
