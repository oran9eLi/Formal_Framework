/** @file adc_hal.h
 * @brief 电流采样测试专用 HAL 类型及调用计数，不影响公共桩。
 */
#ifndef TEST_ADC_HAL_H
#define TEST_ADC_HAL_H
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { unsigned ClockPrescaler, Resolution, ScanConvMode, ContinuousConvMode,
  DiscontinuousConvMode, ExternalTrigConvEdge, ExternalTrigConv, DataAlign,
  NbrOfConversion, DMAContinuousRequests, EOCSelection; } ADC_InitTypeDef;
typedef struct { void *Instance; ADC_InitTypeDef Init; } ADC_HandleTypeDef;
typedef struct { unsigned Channel, Rank, SamplingTime, Offset; } ADC_ChannelConfTypeDef;
typedef struct { unsigned Pin, Mode, Pull, Speed; } GPIO_InitTypeDef;
typedef unsigned GPIO_TypeDef;
#define ADC3 ((void *)3)
#define GPIOC ((GPIO_TypeDef *)3)
#define ADC_CHANNEL_10 10U
#define ADC_CHANNEL_11 11U
#define GPIO_PIN_0 1U
#define GPIO_PIN_1 2U
#define ADC_SAMPLETIME_480CYCLES 480U
#define ADC_CLOCK_SYNC_PCLK_DIV4 4U
#define ADC_RESOLUTION_12B 12U
#define DISABLE 0U
#define ADC_EXTERNALTRIGCONVEDGE_NONE 0U
#define ADC_SOFTWARE_START 0U
#define ADC_DATAALIGN_RIGHT 0U
#define ADC_EOC_SINGLE_CONV 0U
#define GPIO_MODE_OUTPUT_PP 1U
#define GPIO_MODE_ANALOG 2U
#define GPIO_NOPULL 0U
#define GPIO_SPEED_FREQ_LOW 0U
#define GPIO_PIN_SET 1U
#define GPIO_PIN_RESET 0U
#define __HAL_RCC_GPIOC_CLK_ENABLE() ((void)0)
#define __NOP() ((void)0)
static unsigned drive_calls, write_calls, adc_calls, pin_mode = GPIO_MODE_ANALOG;
static unsigned adc_last_channel, adc_channel_configs[32];
/** @brief 各ADC实例保留各自通道，不能让ADC1参考测量覆盖ADC2/ADC3的硬件选择事实。 */
static unsigned adc_instance_channels[4], adc_instance_configured[4];
static GPIO_TypeDef *adc_gpio_last_port;
static unsigned adc_gpio_last_pin;
static HAL_StatusTypeDef adc_result = HAL_OK;
static HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *h) { (void)h; return HAL_OK; }
static HAL_StatusTypeDef HAL_ADC_DeInit(ADC_HandleTypeDef *h) { (void)h; return HAL_OK; }
static HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c)
{
  uintptr_t instance=(uintptr_t)h->Instance;
  adc_last_channel=c->Channel;
  if(c->Channel<32U) { ++adc_channel_configs[c->Channel]; }
  if(instance<4U) { adc_instance_channels[instance]=c->Channel;adc_instance_configured[instance]=1U; }
  return HAL_OK;
}
static HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *h)
{ (void)h; ++adc_calls; assert(pin_mode == GPIO_MODE_ANALOG); return adc_result; }
static HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *h) { (void)h; return HAL_OK; }
static HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *h, uint32_t ms)
{ (void)h; (void)ms; return HAL_OK; }
static uint32_t HAL_ADC_GetValue(ADC_HandleTypeDef *h)
{
  uintptr_t instance=(uintptr_t)h->Instance;
  unsigned channel=(instance<4U && adc_instance_configured[instance])?adc_instance_channels[instance]:adc_last_channel;
  return (channel==17U)?1500U:1000U;
}
static inline void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *g)
{ adc_gpio_last_port=p;adc_gpio_last_pin=g->Pin;pin_mode=g->Mode;if(pin_mode==GPIO_MODE_OUTPUT_PP) { ++drive_calls; } }
static inline void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t pin, unsigned value)
{ (void)p; (void)pin; (void)value; ++write_calls; }
#endif
