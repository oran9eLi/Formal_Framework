/**
 * @file test_single_supply_adc.c
 * @brief 主机验证真实 ADC1/ADC2/ADC3 实现的单组主回路采样与共享内部参考。
 * @details HAL 为计数桩；确认禁用 PA5/PC0 不转换、不驱动，PA4/PC1 仍使用 ADC1 VREFINT。
 */
#include <assert.h>
#include <stdio.h>
#include "runtime_stubs/bsp_critical.h"
#define BSP_CRITICAL_H
#define BSP_ENABLE_POWER1 0U
#define BSP_ENABLE_POWER2 1U
#include "runtime_stubs/adc_hal.h"
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
static const uint16_t test_cal = 1500U;
#define BSP_ADC_VREFINT_CAL test_cal
#include "../../Bsp/Src/bsp_adc.c"
#include "../../Bsp/Src/bsp_adc2.c"
#include "../../Bsp/Src/bsp_adc_current.c"

/** @brief 禁用接口不得触发惰性初始化、转换或修改调用者的旧输出。 */
static void CheckDisabled(void)
{
  uint32_t value = 123456U;
  unsigned count = adc_calls;
  assert(BSP_ADC_ReadRaw(&value) == BSP_STATUS_ERROR);
  assert(BSP_ADC_ReadAverage(&value, 16U) == BSP_STATUS_ERROR);
  assert(BSP_ADC_ReadVoltageMv(&value) == BSP_STATUS_ERROR);
  assert(BSP_ADC_Current_ReadRaw(BSP_ADC_CURRENT_BATTERY1, &value) == BSP_STATUS_ERROR);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY1, &value) == BSP_STATUS_ERROR);
  assert(value == 123456U && adc_calls == count);
  assert(drive_calls == 0U && write_calls == 0U);
}

int main(void)
{
  uint32_t value, expected;
  CheckDisabled();
  assert(BSP_ADC_GetVddaMv() == 3300U);
  assert(adc_channel_configs[ADC_CHANNEL_VREFINT] > 0U);
  assert(BSP_ADC2_ReadVoltageMv(&value) == BSP_STATUS_OK);
  expected = (806U * BSP_ADC2_DIVIDER_NUM) / BSP_ADC2_DIVIDER_DEN;
  assert(value == expected && adc_channel_configs[ADC_CHANNEL_4] > 0U);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2, &value) == BSP_STATUS_OK);
  assert(value == 806U && adc_channel_configs[ADC_CHANNEL_11] > 0U);
  CheckDisabled();
  assert(adc_channel_configs[ADC_CHANNEL_5] == 0U);
  assert(adc_channel_configs[ADC_CHANNEL_10] == 0U);
  assert(drive_calls == 0U && write_calls == 0U);
  assert(BSP_ADC2_Recover() == BSP_STATUS_OK);
  assert(BSP_ADC_Current_Recover() == BSP_STATUS_OK);
  assert(BSP_ADC2_ReadVoltageMv(&value) == BSP_STATUS_OK);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2, &value) == BSP_STATUS_OK);
  puts("single_supply_adc: PASS (VREFINT retained, PA5/PC0 disabled, PA4/PC1 and recovery valid)");
  return 0;
}
