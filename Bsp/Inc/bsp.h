/**
 * @file bsp.h
 * @brief 声明板级统一初始化入口。
 */

#ifndef BSP_H
#define BSP_H

#include "bsp_status.h"

/**
 * @brief 从统一入口初始化所有启用的板级外设。
 *
 * @details
 * 这是唯一板级集中初始化路径。传感器驱动和平台适配器不应重复调用各个
 * BSP_*_Init()，而应依赖外设已在这里初始化。
 */
BSP_Status_t BSP_Init(void);

#endif
