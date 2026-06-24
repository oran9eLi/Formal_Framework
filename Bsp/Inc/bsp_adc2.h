/**
 * @file bsp_adc2.h
 * @brief 声明用于第二块电池电源采样的板级 ADC2 接口。
 *
 * @details
 * 本模块参照 bsp_adc(ADC1/PA5) 实现，使用独立的 ADC2 外设(PA4/ADC2_IN4)，
 * 与电池 1 采样在硬件上完全独立、互不影响。第二块电池与第一块同规格，
 * 分压系数相同。
 */

#ifndef BSP_ADC2_H
#define BSP_ADC2_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief 初始化板级 ADC2 电源采样通道。
 */
BSP_Status_t BSP_ADC2_Init(void);

/**
 * @brief 在转换故障后重新初始化 ADC2。
 */
BSP_Status_t BSP_ADC2_Recover(void);

/**
 * @brief 释放 ADC2 外设资源。
 */
BSP_Status_t BSP_ADC2_DeInit(void);

/**
 * @brief 读取一次原始 ADC2 转换值。
 */
BSP_Status_t BSP_ADC2_ReadRaw(uint32_t *raw);

/**
 * @brief 读取多次 ADC2 转换并返回四舍五入平均值。
 */
BSP_Status_t BSP_ADC2_ReadAverage(uint32_t *raw, uint8_t count);

/**
 * @brief 读取分压还原后的第二块电池电压，单位 mV。
 */
BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *voltage_mv);

#endif
