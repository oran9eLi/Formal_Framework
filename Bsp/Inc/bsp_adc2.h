/**
 * @file bsp_adc2.h
 * @brief 声明用于第二块电池电源采样的板级 ADC2 接口。
 *
 * @details
 * 本模块使用ADC2监测所选主回路分压引脚（扩展板PA5或独立直连PA4），独立于ADC1采样通道，
 * 但复用ADC1内部参考估算公共模拟参考。只负责原始采样与mV换算，不解释电源来源。
 * 可选所选电压脚输入弱下拉dummy采样后先恢复Analog，再合作等待外部源恢复；不判定模块在位。
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
  uint32_t discharge_raw;        /**< 输入弱下拉时dummy平均12bit原码，仅为诊断，不作为正式电压。 */
  uint32_t discharge_pin_mv;     /**< dummy原码按正式同轮VDDA换算的引脚mV；仅本轮sample_valid时解释。 */
  uint16_t recovery_ms;          /**< 本轮已配置的模拟输入恢复窗口，单位ms；关闭弱泄放时为0。 */
  uint8_t discharge_pending;     /**< 1为已恢复Analog并等待恢复窗口；不是正在持续下拉。 */
  uint8_t discharge_status;      /**< dummy采样的原始BSP_Status_t状态；正式采样状态另看sample_status。 */
  uint8_t discharge_enabled;     /**< 1为本轮明确启用弱泄放；0不能解释为已检测到输入在位。 */
  uint8_t discharge_checked;     /**< 1为已成功取得弱下拉dummy原码；不能单独证明模块连接或供电。 */
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
 * @brief 合作读取分压还原后的第二组电压，单位mV；仅正式Analog采样成功才改写输出。
 * @retval BSP_STATUS_BUSY 弱泄放后已经恢复Analog，需由原sensor所有者稍后重入推进。
 * @retval BSP_STATUS_OK 已完成恢复窗口与正常参考/16次电压采样。
 * @retval BSP_STATUS_ERROR 参数、初始化或转换失败；硬件HAL_BUSY不冒充合作等待。
 * @retval BSP_STATUS_TIMEOUT HAL转换超时，诊断保留原状态且不改写输出。
 * @note 关闭弱泄放时保留一次调用读取；启用时无忙等/延时，不驱动推挽输出或电流脚。
 *       默认恢复窗口只是工程候选，真实源阻抗/外接电容的准确性必须实板验证。
 */
BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *voltage_mv);

#endif
