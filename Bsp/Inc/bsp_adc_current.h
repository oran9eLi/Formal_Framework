/**
 * @file bsp_adc_current.h
 * @brief 声明两路电流计模拟输出的板级 ADC 接口。
 * @details
 * 本文件属于 BSP 层，只负责 PC0/PC1 ADC 采样和引脚电压换算。电流计零点、比例、
 * 滤波、功率和过流判断由 Sensor Power 层完成，上层不得绕过 Sensor 直接解释 ADC 值。
 */

#ifndef BSP_ADC_CURRENT_H
#define BSP_ADC_CURRENT_H

#include "bsp_status.h"
#include <stdint.h>

/**
 * @brief 电流计 ADC 通道。
 */
typedef enum {
  BSP_ADC_CURRENT_BATTERY1 = 0, /**< 第一电池电流计输出，PC0 / ADC_IN10。 */
  BSP_ADC_CURRENT_BATTERY2 = 1  /**< 第二电池电流计输出，PC1 / ADC_IN11。 */
} BSP_ADC_CurrentChannel_t;

/**
 * @brief 初始化两路电流计 ADC。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_Init(void);

/**
 * @brief 在转换故障后重新初始化电流计 ADC。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_Recover(void);

/**
 * @brief 释放电流计 ADC 外设资源。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_DeInit(void);

/**
 * @brief 读取指定电流计通道的一次原始 ADC 值。
 * @param[in] channel 电流计通道。
 * @param[out] raw 输出原始 ADC 值，范围 0 到 4095。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_ReadRaw(BSP_ADC_CurrentChannel_t channel, uint32_t *raw);

/**
 * @brief 读取指定电流计通道多次 ADC 值并返回平均值。
 * @param[in] channel 电流计通道。
 * @param[out] raw 输出平均后的原始 ADC 值。
 * @param[in] count 采样次数，0 表示使用 1 次。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_ReadAverage(BSP_ADC_CurrentChannel_t channel, uint32_t *raw, uint8_t count);

/**
 * @brief 读取指定电流计通道的 ADC 引脚电压。
 * @param[in] channel 电流计通道。
 * @param[out] voltage_mv 输出 ADC 引脚电压，单位 mV。
 * @return BSP 通用返回码。
 */
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv);

/**
 * @brief 读取上一次 ReadVoltageMv 缓存的诊断值（不访问 ADC，仅供调试打印）。
 * @param[in] channel 电流计通道。
 * @param[out] adc_mv 上次采样的原始引脚电压（mV）；浮空判定命中时为 0。可为 NULL。
 * @param[out] floating 上次是否被判为浮空（1=浮空强制清零，0=读到真实电压）。可为 NULL。
 * @note 临时诊断接口，定位电流计接线后可移除。
 */
void BSP_ADC_Current_GetDiag(BSP_ADC_CurrentChannel_t channel, uint32_t *adc_mv, uint8_t *floating);

#endif
