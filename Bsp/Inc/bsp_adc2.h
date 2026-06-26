/**
 * @file bsp_adc2.h
 * @brief 声明用于第二块电池采样的板级 ADC2 接口。
 *
 * @details
 * 本文件属于 BSP 层，只暴露 PA4/ADC2_IN4 的原始采样和分压还原接口。上层不得绕过
 * Sensor Power2 驱动直接解释电池状态。
 */

#ifndef BSP_ADC2_H
#define BSP_ADC2_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief 初始化第二块电池采样 ADC2 通道。
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
