/**
 * @file bsp_adc.h
 * @brief 声明用于电源采样的板级 ADC 接口。
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#include "bsp_status.h"

/**
 * @brief 初始化板级 ADC 电源采样通道。
 */
BSP_Status_t BSP_ADC_Init(void);

/**
 * @brief 在转换故障后重新初始化 ADC。
 */
BSP_Status_t BSP_ADC_Recover(void);

/**
 * @brief 释放 ADC 外设资源。
 */
BSP_Status_t BSP_ADC_DeInit(void);

/**
 * @brief 读取一次原始 ADC 转换值。
 */
BSP_Status_t BSP_ADC_ReadRaw(uint32_t *raw);

/**
 * @brief 读取多次 ADC 转换并返回四舍五入平均值。
 */
BSP_Status_t BSP_ADC_ReadAverage(uint32_t *raw, uint8_t count);

/**
 * @brief 读取分压还原后的输入电压，单位 mV。
 */
BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv);

/**
 * @brief 用内部 VREFINT 实测当前 VDDA(模拟供电)电压，单位 mV。
 *
 * @details VREFINT 仅接 ADC1，故由本模块测量；VDDA 为 ADC1/ADC2 公共供电，
 *          ADC2 也应使用此返回值做比例换算。读取失败时返回标称 3300mV。
 */
uint32_t BSP_ADC_GetVddaMv(void);

#endif
