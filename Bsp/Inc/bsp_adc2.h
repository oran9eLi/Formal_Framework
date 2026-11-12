/**
 * @file bsp_adc2.h
 * @brief 声明用于第二块电池电源采样的板级 ADC2 接口。
 *
 * @details
 * 本模块使用ADC2/PA4监测主回路分压信号，独立于ADC1采样通道，
 * 但复用ADC1内部参考估算公共模拟参考。只负责原始采样与mV换算，不解释电源来源。
 */

#ifndef BSP_ADC2_H
#define BSP_ADC2_H

#include <stdint.h>

#include "bsp_status.h"
#include "bsp_adc.h"

/** @brief 最近一次ADC2电压尝试及它实际使用的参考副本，不被后续ADC3参考采样覆盖。 */
typedef struct {
  uint32_t sequence;             /**< 电压采集尝试序号，uint32自然回绕。 */
  uint32_t raw_average;          /**< 同16次采样的四舍五入平均12bit原码，仅sample_valid时有效。 */
  uint32_t raw_min;              /**< 同轮原码最小值，不额外采样。 */
  uint32_t raw_max;              /**< 同轮原码最大值，不额外采样。 */
  uint32_t pin_mv;               /**< ADC输入脚估算mV，仅sample_valid时解释。 */
  uint32_t input_mv;             /**< 当前配置分压还原后的主回路mV，非Sensor二次校准值。 */
  BSP_ADC_ReferenceDiag_t reference; /**< 同次电压实际采用的参考缓存，可能明确回退。 */
  uint8_t ready;                 /**< 已记录一次电压尝试为1，初始化/恢复不擦历史证据。 */
  uint8_t sample_valid;          /**< ADC2采样成功为1，不等于reference为实测；回退另看reference.status。 */
  uint8_t sample_status;         /**< 本次BSP_Status_t结果，失败时原ReadVoltage输出保持不变。 */
} BSP_ADC2_Diag_t;

/** @brief 只复制缓存，不读ADC；NULL或无尝试返回ERROR且不改输出。 */
BSP_Status_t BSP_ADC2_GetDiag(BSP_ADC2_Diag_t *out);

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
