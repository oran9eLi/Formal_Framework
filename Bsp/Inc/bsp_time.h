/**
 * @file bsp_time.h
 * @brief 声明设备驱动使用的板级时间接口。
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/**
 * @brief 返回板级毫秒 tick。
 */
uint32_t BSP_Time_GetTickMs(void);

/**
 * @brief 延时指定毫秒数。
 */
void BSP_Time_DelayMs(uint32_t delay_ms);

#endif
