/**
 * @file test_power_adc_reference.c
 * @brief 验证生产 BSP 的参考失效可见性与第二组电压诊断的一致性。
 * @details 本文件属于主机测试层，依赖 HAL 替身和真实 BSP；不证明实际电流计标定通过。
 */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#define BSP_CRITICAL_H
static uint32_t s_critical_depth;

/** @brief 记录快照临界区深度，便于检出在关中断期间访问硬件的边界违规。 */
static uint32_t BSP_Critical_Enter(void) { return s_critical_depth++; }

/** @brief 模拟按原中断状态恢复，禁止遗漏或不配对的退出。 */
static void BSP_Critical_Exit(uint32_t saved)
{
  assert(s_critical_depth == saved + 1U);
  s_critical_depth = saved;
}
#define BSP_ENABLE_POWER1 0U
#define BSP_ENABLE_POWER2 1U
#define HAL_ADC_GetValue TestDefaultAdcValue
#define HAL_ADC_Start TestDefaultAdcStart
#include "runtime_stubs/adc_hal.h"
#undef HAL_ADC_GetValue
#undef HAL_ADC_Start
#define HAL_BUSY 2U
#define HAL_TIMEOUT 3U
#define ADC1 ((void *)1)
#define ADC2 ((void *)2)
#define GPIOA ((GPIO_TypeDef *)1)
#define ADC_CHANNEL_4 4U
#define ADC_CHANNEL_5 5U
#define ADC_CHANNEL_VREFINT 17U
#define GPIO_PIN_4 (1U << 4)
#define GPIO_PIN_5 (1U << 5)
#define __HAL_RCC_GPIOA_CLK_ENABLE() ((void)0)

static uint32_t s_raw_vref = 1500U;
static uint32_t s_raw_voltage = 1464U;
static uint16_t s_factory_cal = 1500U;
static uint8_t s_fail_vref;
static uint8_t s_fail_voltage;
#define BSP_ADC_VREFINT_CAL s_factory_cal

/** @brief 分别注入两个 ADC 的独立采样输入，避免将公式输出直接作为测试输入。 */
static uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *handle)
{
  assert(s_critical_depth == 0U);
  if (handle->Instance == ADC1) { return s_raw_vref; }
  if (handle->Instance == ADC2) { return s_raw_voltage; }
  return TestDefaultAdcValue(handle);
}

/** @brief 独立模拟参考采集和电压采集失败，验证缓存不会把失败伪装成新测量。 */
static HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *handle)
{
  assert(s_critical_depth == 0U);
  if (((handle->Instance == ADC1) && s_fail_vref) ||
      ((handle->Instance == ADC2) && s_fail_voltage)) { return HAL_ERROR; }
  return TestDefaultAdcStart(handle);
}

#include "../../Bsp/Src/bsp_adc.c"
#include "../../Bsp/Src/bsp_adc2.c"

/** @brief 失效参考仍返回标称电压时，必须通过诊断明确标记其回退原因。 */
static void CheckReferenceFallback(uint16_t calibration, uint32_t raw, uint8_t failure,
                                   uint8_t expected_status)
{
  BSP_ADC_ReferenceDiag_t diag;
  unsigned conversions;
  s_factory_cal = calibration;
  s_raw_vref = raw;
  s_fail_vref = failure;
  assert(BSP_ADC_GetVddaMv() == 3300U);
  conversions = adc_calls;
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_OK);
  assert(diag.ready == 1U && diag.sequence != 0U);
  assert(diag.status == expected_status);
  assert(diag.calibration_raw == calibration && diag.vdda_mv == 3300U);
  assert(adc_calls == conversions);
  s_fail_vref = 0U;
}

/** @brief 确认合法供电变化会被实测跟踪，不会把所有输入固定为标称 3.3V。 */
static void CheckMeasuredReference(uint32_t raw, uint32_t expected_mv)
{
  BSP_ADC_ReferenceDiag_t diag;
  s_factory_cal = 1500U;
  s_raw_vref = raw;
  assert(BSP_ADC_GetVddaMv() == expected_mv);
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_OK);
  assert(diag.status == BSP_ADC_REF_MEASURED && diag.ready == 1U);
  assert(diag.vref_raw == raw && diag.calibration_raw == 1500U);
  assert(diag.vdda_mv == expected_mv && diag.calculated_vdda_mv == expected_mv);
}

/** @brief 覆盖正常 1.18V 信号、参考故障矩阵和无额外硬件访问的缓存读取。 */
int main(void)
{
  BSP_ADC_ReferenceDiag_t diag;
  BSP_ADC2_Diag_t voltage_diag;
  uint32_t voltage_mv = 0U;
  uint32_t sequence;
  unsigned conversions;
  assert(BSP_ADC_GetReferenceDiag(0) == BSP_STATUS_ERROR);
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_ERROR);
  assert(BSP_ADC2_GetDiag(0) == BSP_STATUS_ERROR);
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_ERROR);
  CheckMeasuredReference(1500U, 3300U);
  CheckMeasuredReference(1650U, 3000U);
  CheckMeasuredReference(1375U, 3600U);
  /* 当前 ADC 时钟为 21MHz；低于 2.4V 不满足此时钟的器件工作条件。 */
  CheckMeasuredReference(2062U, 2400U);

  CheckReferenceFallback(0U, 1500U, 0U, BSP_ADC_REF_FALLBACK_CAL_ZERO);
  CheckReferenceFallback(65535U, 1500U, 0U, BSP_ADC_REF_FALLBACK_CAL_ERASED);
  CheckReferenceFallback(4096U, 1500U, 0U, BSP_ADC_REF_FALLBACK_CAL_RANGE);
  CheckReferenceFallback(1500U, 1500U, 1U, BSP_ADC_REF_FALLBACK_READ);
  CheckReferenceFallback(1500U, 0U, 0U, BSP_ADC_REF_FALLBACK_RAW);
  CheckReferenceFallback(1500U, 4096U, 0U, BSP_ADC_REF_FALLBACK_RAW);
  CheckReferenceFallback(1500U, 1000U, 0U, BSP_ADC_REF_FALLBACK_VDDA_RANGE);
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_OK);
  assert(diag.calculated_vdda_mv == 4950U);
  CheckReferenceFallback(1500U, 4000U, 0U, BSP_ADC_REF_FALLBACK_VDDA_RANGE);
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_OK);
  assert(diag.calculated_vdda_mv < 1800U);
  CheckReferenceFallback(1500U, 2750U, 0U, BSP_ADC_REF_FALLBACK_VDDA_RANGE);
  assert(BSP_ADC_GetReferenceDiag(&diag) == BSP_STATUS_OK);
  assert(diag.calculated_vdda_mv == 1800U);

  s_raw_vref = 1500U;
  s_factory_cal = 1500U;
  s_raw_voltage = 1464U;
  assert(BSP_ADC2_ReadVoltageMv(&voltage_mv) == BSP_STATUS_OK);
  assert(voltage_mv >= 12000U && voltage_mv <= 12020U);
  conversions = adc_calls;
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(adc_calls == conversions);
  assert(voltage_diag.ready == 1U && voltage_diag.sample_valid == 1U);
  assert(voltage_diag.sample_status == BSP_STATUS_OK);
  assert(voltage_diag.raw_average == 1464U && voltage_diag.raw_min == 1464U && voltage_diag.raw_max == 1464U);
  assert(voltage_diag.pin_mv == 1180U && voltage_diag.input_mv == voltage_mv);
  assert(voltage_diag.reference.status == BSP_ADC_REF_MEASURED);
  assert(voltage_diag.reference.vdda_mv == 3300U && voltage_diag.reference.vref_raw == 1500U);
  sequence = voltage_diag.sequence;
  voltage_diag.reference.vdda_mv = 0U;
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(voltage_diag.reference.vdda_mv == 3300U && voltage_diag.sequence == sequence);
  assert(adc_calls == conversions);

  /* 后续 ADC3 等消费者可能重新测参考；ADC2 缓存必须保留其同轮参考而非全局最新值。 */
  CheckMeasuredReference(1650U, 3000U);
  conversions = adc_calls;
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(adc_calls == conversions);
  assert(voltage_diag.reference.vdda_mv == 3300U && voltage_diag.reference.vref_raw == 1500U);
  assert(voltage_diag.sequence == sequence);

  /* 失败尝试必须标记无效，不得改写调用者输出或继续伪装上一帧为新采集。 */
  s_fail_voltage = 1U;
  voltage_mv = 123456U;
  assert(BSP_ADC2_ReadVoltageMv(&voltage_mv) == BSP_STATUS_ERROR);
  assert(voltage_mv == 123456U);
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(voltage_diag.sample_valid == 0U && voltage_diag.sample_status != BSP_STATUS_OK);
  assert(voltage_diag.sequence != sequence);
  s_fail_voltage = 0U;
  s_raw_voltage = 4096U;
  assert(BSP_ADC2_ReadVoltageMv(&voltage_mv) == BSP_STATUS_ERROR);
  assert(voltage_mv == 123456U);
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(voltage_diag.sample_valid == 0U);

  /* 参考回退允许维持旧兼容接口，但诊断必须随同该次读数明确指出未使用实测参考。 */
  s_factory_cal = 65535U;
  s_raw_voltage = 1464U;
  assert(BSP_ADC2_ReadVoltageMv(&voltage_mv) == BSP_STATUS_OK);
  assert(BSP_ADC2_GetDiag(&voltage_diag) == BSP_STATUS_OK);
  assert(voltage_diag.sample_valid == 1U);
  assert(voltage_diag.reference.status == BSP_ADC_REF_FALLBACK_CAL_ERASED);
  assert(voltage_diag.reference.vdda_mv == 3300U);
  assert(drive_calls == 0U && write_calls == 0U);
  assert(adc_channel_configs[ADC_CHANNEL_5] == 0U);
  assert(s_critical_depth == 0U);
  puts("power ADC reference: PASS (valid tracking; fallback matrix; coherent read-only diag; 1.18V -> 12.01V)");
  return 0;
}
