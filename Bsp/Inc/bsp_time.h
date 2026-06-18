/**
 * @file bsp_time.h
 * @brief Declare board time helpers used by device drivers.
 */

#ifndef BSP_TIME_H
#define BSP_TIME_H

#include <stdint.h>

/**
 * @brief Return the board millisecond tick.
 */
uint32_t BSP_Time_GetTickMs(void);

/**
 * @brief Delay for a bounded number of milliseconds.
 */
void BSP_Time_DelayMs(uint32_t delay_ms);

#endif
