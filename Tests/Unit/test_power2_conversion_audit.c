/**
 * @file test_power2_conversion_audit.c
 * @brief 主机审计第二组真实 ADC 到电压、电流、功率的单位与参考换算。
 * @details BSP 与 Sensor 使用生产实现，HAL 注入已知原始码；不据模拟结果声明实物标定通过。
 */
#include <assert.h>
#include <stdio.h>
#include "runtime_stubs/bsp_critical.h"
#define BSP_CRITICAL_H
#define BSP_ENABLE_POWER1 0U
#define BSP_ENABLE_POWER2 1U
#define HAL_ADC_GetValue TestDefaultAdcValue
#include "runtime_stubs/adc_hal.h"
#undef HAL_ADC_GetValue
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
static uint32_t s_raw_voltage = 1489U;
static uint32_t s_raw_current = 65U;
static const uint16_t s_factory_cal = 1500U;
#define BSP_ADC_VREFINT_CAL s_factory_cal

/** @brief 仅注入硬件原始码，生产代码仍负责所有参考、分压与物理单位换算。 */
static uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *handle)
{
  if ((handle->Instance == ADC1) && (adc_last_channel == ADC_CHANNEL_VREFINT)) { return s_raw_vref; }
  if (handle->Instance == ADC2) { return s_raw_voltage; }
  if (handle->Instance == ADC3) { return s_raw_current; }
  return TestDefaultAdcValue(handle);
}

#include "../../Bsp/Src/bsp_adc.c"
#include "../../Bsp/Src/bsp_adc2.c"
#include "../../Bsp/Src/bsp_adc_current.c"
/* 独立翻译单元内的静态名字合并，仅在测试夹具重命名；生产文件分别编译。 */
#define s_initialized2 s_power_driver_initialized2
#include "../../Sensor/Src/sensor_power2.c"
#undef s_initialized2

/** @brief 每个独立工况从真实初始化与Service入口采集，避免旧滤波值影响单位审计。 */
static Power_Snapshot_t ReadCondition(uint32_t voltage_raw, uint32_t current_raw, uint32_t vref_raw)
{
  Power_Snapshot_t sample;
  s_raw_voltage = voltage_raw;
  s_raw_current = current_raw;
  s_raw_vref = vref_raw;
  assert(Sensor_Power2_Init() == POWER_RESULT_OK);
  assert(Sensor_Power2_Service(1000U) == POWER_RESULT_OK);
  assert(Sensor_Power2_CopySnapshot(&sample) == POWER_RESULT_OK);
  assert(sample.current_valid == 1U);
  return sample;
}

/** @brief 已知输入证明0.8 A不会因mA/A单位错误被变成16 A，公共VDDA变化仍正确跟踪。 */
int main(void)
{
  Power_Snapshot_t sample = ReadCondition(1489U, 65U, 1500U);
  assert((uint32_t)(sample.voltage_v * 1000.0f + 0.5f) == 12213U);
  assert(sample.current_ma == 800 && sample.power_mw == 9770U);
  /* 同一1200mV电压信号及约52mV电流信号，在VDDA=3.0V时仍还原同样的V/A/W。 */
  sample = ReadCondition(1638U, 71U, 1650U);
  assert((uint32_t)(sample.voltage_v * 1000.0f + 0.5f) == 12213U);
  assert(sample.current_ma == 800 && sample.power_mw == 9770U);
  /* 用户本次实测 PA4 约 1.18V；在正常参考下应为约 12.01V，不能变成 7–8V。 */
  sample = ReadCondition(1464U, 65U, 1500U);
  assert((uint32_t)(sample.voltage_v * 1000.0f + 0.5f) >= 12000U);
  assert((uint32_t)(sample.voltage_v * 1000.0f + 0.5f) <= 12020U);
  assert(sample.current_ma == 800);
  /* 显示偏差可由不同的ADC信号重现；这不证明实物引脚就是这些电压。 */
  sample = ReadCondition(1013U, 1331U, 1500U);
  assert((uint32_t)(sample.voltage_v * 1000.0f + 0.5f) == 8305U);
  assert(sample.current_ma == 16507);
  assert(adc_channel_configs[ADC_CHANNEL_5] == 0U && adc_channel_configs[ADC_CHANNEL_10] == 0U);
  assert(drive_calls == 0U && write_calls == 0U);
  puts("power2 conversion audit: PASS (12.213 V / 0.800 A -> 9.770 W; VDDA tracking; observed-scale reproduction)");
  return 0;
}
