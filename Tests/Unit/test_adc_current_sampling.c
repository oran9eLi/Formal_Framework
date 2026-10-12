/** @file test_adc_current_sampling.c
 * @brief 真实 ADC 电流采样：正常采样不驱动引脚，诊断探测受完整开关控制。
 */
#include <assert.h>
#include <stdio.h>
#include "runtime_stubs/adc_hal.h"
#include "../../Bsp/Src/bsp_adc_current.c"
uint32_t BSP_ADC_GetVddaMv(void) { return 3300U; }
int main(void)
{
  uint32_t raw, mv = 123U;
  assert(BSP_ADC_Current_ReadRaw(BSP_ADC_CURRENT_BATTERY1, &raw) == BSP_STATUS_OK);
  assert(raw == 1000U);
  assert(drive_calls == 0U && write_calls == 0U);
  assert(BSP_ADC_Current_ReadAverage(BSP_ADC_CURRENT_BATTERY2, &raw, 4U) == BSP_STATUS_OK);
  assert(drive_calls == 0U && write_calls == 0U);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY1, &mv) == BSP_STATUS_OK);
  assert(mv == 806U);
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  assert(drive_calls == 2U && write_calls == 2U);
#else
  assert(drive_calls == 0U && write_calls == 0U);
#endif
  drive_calls = write_calls = adc_calls = 0U;
  assert(BSP_ADC_Current_ReadVoltageMv((BSP_ADC_CurrentChannel_t)9, &mv) == BSP_STATUS_ERROR);
  assert(adc_calls == 0U && drive_calls == 0U);
  adc_result = HAL_ERROR;
  mv = 123U;
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY1, &mv) == BSP_STATUS_ERROR);
  assert(mv == 123U && pin_mode == GPIO_MODE_ANALOG);
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  assert(adc_calls == 1U && drive_calls == 1U);
#else
  assert(drive_calls == 0U);
#endif
  puts("ADC current sampling tests passed");
  return 0;
}
