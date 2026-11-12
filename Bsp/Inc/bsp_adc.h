/**
 * @file bsp_adc.h
 * @brief BSP 层 ADC1 内部参考与可选第一组电压采样接口。
 * @details 依赖 BSP 状态类型；禁用第一组时外部读取返回错误且不触碰 PA5。
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#include "bsp_status.h"

/** @brief VREFINT参考结果；回退仍返回兼容标称值，不能称为本次实测参考。 */
typedef enum {
  BSP_ADC_REF_MEASURED = 0,          /**< 工厂码、原码及计算参考均合法。 */
  BSP_ADC_REF_FALLBACK_CAL_ZERO,     /**< 工厂校准字为0，未采参考。 */
  BSP_ADC_REF_FALLBACK_CAL_ERASED,   /**< 工厂校准字为0xFFFF，未采参考。 */
  BSP_ADC_REF_FALLBACK_CAL_RANGE,    /**< 工厂校准字超出12bit范围，未采参考。 */
  BSP_ADC_REF_FALLBACK_READ,         /**< VREFINT配置/转换失败。 */
  BSP_ADC_REF_FALLBACK_RAW,          /**< VREFINT原码为0或超出12bit范围。 */
  BSP_ADC_REF_FALLBACK_VDDA_RANGE    /**< 计算参考超当前允许窗口；保留计算值但使用回退。 */
} BSP_ADC_ReferenceStatus_t;

/**
 * @brief 最近一次owner参考测量尝试缓存；非硬件实时寄存器视图。
 * @details 默认允许2400..3600mV：当前ADC时钟21MHz，低于2.4V的STM32F407仅允许最高18MHz。
 *          未来改变板级供电/ADC时钟时可覆盖BSP_ADC_VDDA_MIN_MV/MAX_MV，不可据屏显比值凑标定。
 */
typedef struct {
  uint32_t sequence;           /**< 参考尝试序号，uint32自然回绕，0须结合ready解释。 */
  uint32_t vref_raw;           /**< VREFINT平均原码；未读/转换失败时0，原码非法时保留异常读数。 */
  uint32_t vdda_mv;            /**< 本次实际用于换算的参考mV，可能为明确标记的3300回退值。 */
  uint32_t calculated_vdda_mv; /**< 未经范围接纳的计算值mV；未计算时0，不能直接授权使用。 */
  uint16_t calibration_raw;    /**< 本次读取的工厂校准字，十六进制原值保留。 */
  uint8_t status;              /**< BSP_ADC_ReferenceStatus_t，0实测/其它回退原因。 */
  uint8_t ready;               /**< 至少完成一次参考尝试为1，warm初始化/恢复不擦证据。 */
} BSP_ADC_ReferenceDiag_t;

/** @brief 只在短临界区复制参考诊断缓存；NULL或未有尝试返回ERROR，不采ADC，不改输出。 */
BSP_Status_t BSP_ADC_GetReferenceDiag(BSP_ADC_ReferenceDiag_t *out);

/**
 * @brief 初始化 ADC1；第一组关闭时仅保留内部 VREFINT，不占用 PA5。
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
 * @brief 读取第一组分压还原电压，单位 mV；该组停用时返回 ERROR，输出不变。
 */
BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv);

/**
 * @brief 用内部 VREFINT 实测当前 VDDA(模拟供电)电压，单位 mV。
 *
 * @details VREFINT 仅接 ADC1，故由本模块测量；VDDA 为 ADC1/ADC2 公共供电，
 *          ADC2也应使用此返回值做比例换算。工厂码、原码、允许窗口或读取异常时返回3300mV，
 *          必须结合GetReferenceDiag区分实测与回退；不得把回退当成参考硬件已验证。
 */
uint32_t BSP_ADC_GetVddaMv(void);

#endif
