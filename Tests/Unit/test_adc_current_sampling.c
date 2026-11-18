/** @file test_adc_current_sampling.c
 * @brief 真实ADC电流采样：验证扩展板PC0与独立线PC1路由、零电流和逐样本饱和拒绝。
 */
#include <assert.h>
#include <stdio.h>
#define HAL_ADC_GetValue AdcDefaultGetValue
#include "runtime_stubs/adc_hal.h"
#undef HAL_ADC_GetValue
static uint32_t s_values[64];
static unsigned s_value_count,s_value_position;
/** @brief 仅注入HAL实际原始码，不替换生产采样/平均/拒绝决策。 */
static uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{ return (s_value_position<s_value_count)?s_values[s_value_position++]:AdcDefaultGetValue(h); }
#include "../../Bsp/Src/bsp_adc_current.c"
uint32_t BSP_ADC_GetVddaMv(void) { return 3300U; }
/** @brief 按外部接线路径验证实际配置通道，组2身份不因复用原第一路接头而改变。 */
static void CheckCurrent2Route(void)
{
#if BSP_POWER2_USE_EXPANSION_ROUTE
  assert(BSP_ADC_CURRENT2_CH==ADC_CHANNEL_10 && BSP_ADC_CURRENT2_PIN==GPIO_PIN_0);
  assert(adc_last_channel==ADC_CHANNEL_10 && adc_instance_channels[3]==ADC_CHANNEL_10);
#else
  assert(BSP_ADC_CURRENT2_CH==ADC_CHANNEL_11 && BSP_ADC_CURRENT2_PIN==GPIO_PIN_1);
  assert(adc_last_channel==ADC_CHANNEL_11 && adc_instance_channels[3]==ADC_CHANNEL_11);
#endif
  assert(BSP_ADC_CURRENT2_PORT==GPIOC);
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  assert(adc_gpio_last_port==GPIOC && adc_gpio_last_pin==BSP_ADC_CURRENT2_PIN);
#endif
}
/** @brief 构造正式样本；显式诊断模式的两个预置读数固定一致，避免把诊断当测量。 */
static void FormalSamples(uint32_t normal,unsigned clip_at,uint32_t clip_value)
{
  unsigned i,prefix=0U;
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  prefix=2U;
#endif
  for(i=0U;i<64U;i++) { s_values[i]=normal; }
  if(prefix) { s_values[0]=s_values[1]=1000U; }
  if(clip_at<16U) { s_values[prefix+clip_at]=clip_value; }
  s_value_count=prefix+16U;s_value_position=0U;
}
/** @brief 单个4095不得被16次均值稀释，拒绝时不改输出、诊断保留3.3V实测原码事实。 */
static void CheckRailAndValidZero(void)
{
  uint32_t mv=777U,diag;uint8_t floating;unsigned prefix=0U;
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  prefix=2U;
#endif
  adc_result=HAL_OK;drive_calls=write_calls=adc_calls=0U;
  FormalSamples(1000U,5U,4095U);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2,&mv)==BSP_STATUS_ERROR);
  assert(mv==777U && s_value_position==prefix+6U && pin_mode==GPIO_MODE_ANALOG);
  BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY2,&diag,&floating);
  assert(diag==3300U && floating==0U);
  CheckCurrent2Route();
#if !BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  assert(drive_calls==0U && write_calls==0U);
#endif
  FormalSamples(1000U,0U,4096U);mv=778U;
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2,&mv)==BSP_STATUS_ERROR && mv==778U);
  BSP_ADC_Current_GetDiag(BSP_ADC_CURRENT_BATTERY2,&diag,0);assert(diag==3301U);
  FormalSamples(0U,16U,0U);mv=999U;
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2,&mv)==BSP_STATUS_OK && mv==0U);
  CheckCurrent2Route();
  FormalSamples(4094U,16U,0U);
  assert(BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CURRENT_BATTERY2,&mv)==BSP_STATUS_OK && mv==3299U);
}
/** @brief 原始接口保留4095诊断，正式平均单次碰rail也失败，失败不改调用者输出。 */
static void CheckAverageRejectsBeforeMean(void)
{
  uint32_t raw=321U;unsigned i;
  s_values[0]=4095U;s_value_count=1U;s_value_position=0U;
  assert(BSP_ADC_Current_ReadRaw(BSP_ADC_CURRENT_BATTERY2,&raw)==BSP_STATUS_OK && raw==4095U);
  for(i=0U;i<16U;i++) { s_values[i]=1000U; }
  s_values[7]=4095U;s_value_count=16U;s_value_position=0U;raw=321U;
  assert(BSP_ADC_Current_ReadAverage(BSP_ADC_CURRENT_BATTERY2,&raw,16U)==BSP_STATUS_ERROR);
  assert(raw==321U && s_value_position==8U);
  s_values[0]=4095U;s_value_count=1U;s_value_position=0U;
  assert(BSP_ADC_Current_ReadAverage(BSP_ADC_CURRENT_BATTERY2,&raw,0U)==BSP_STATUS_ERROR && raw==321U);
}
int main(void)
{
  uint32_t raw, mv = 123U;
  const BSP_ADC_CurrentChannel_t first_channel=BSP_ENABLE_POWER1?BSP_ADC_CURRENT_BATTERY1:BSP_ADC_CURRENT_BATTERY2;
#if BSP_ENABLE_POWER1
  assert(BSP_ADC_Current_ReadRaw(BSP_ADC_CURRENT_BATTERY1, &raw) == BSP_STATUS_OK);
  assert(raw == 1000U);
#else
  raw=987U;
  assert(BSP_ADC_Current_ReadRaw(BSP_ADC_CURRENT_BATTERY1,&raw)==BSP_STATUS_ERROR && raw==987U);
  assert(adc_calls==0U);
#endif
  assert(drive_calls == 0U && write_calls == 0U);
  assert(BSP_ADC_Current_ReadAverage(BSP_ADC_CURRENT_BATTERY2, &raw, 4U) == BSP_STATUS_OK);
  assert(drive_calls == 0U && write_calls == 0U);
  /* 正常平均也必须实际选择当前BSP路由；不靠测试伪造的通道名证明接入正确。 */
#if BSP_POWER2_USE_EXPANSION_ROUTE
  assert(adc_last_channel==ADC_CHANNEL_10);
#else
  assert(adc_last_channel==ADC_CHANNEL_11);
#endif
  assert(BSP_ADC_Current_ReadVoltageMv(first_channel, &mv) == BSP_STATUS_OK);
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
  assert(BSP_ADC_Current_ReadVoltageMv(first_channel, &mv) == BSP_STATUS_ERROR);
  assert(mv == 123U && pin_mode == GPIO_MODE_ANALOG);
#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  assert(adc_calls == 1U && drive_calls == 1U);
#else
  assert(drive_calls == 0U);
#endif
  CheckRailAndValidZero();CheckAverageRejectsBeforeMean();
  printf("ADC current: route=%u current2_ch=%u pin=%u / no normal GPIO drive / true0 / single4095rail and4096reject / unchanged output / diagnostics PASS\n",
    (unsigned)BSP_POWER2_USE_EXPANSION_ROUTE,(unsigned)BSP_ADC_CURRENT2_CH,(unsigned)BSP_ADC_CURRENT2_PIN);
  return 0;
}
