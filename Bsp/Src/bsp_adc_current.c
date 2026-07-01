/**
 * @file bsp_adc_current.c
 * @brief 实现 PC0/PC1 两路电流计模拟输出采样。
 * @details
 * 本文件属于 BSP 层，只负责 ADC3 初始化、通道切换、轮询采样、平均采样和 ADC 引脚
 * 电压换算。不得在本层解释电流计比例、零点或电池状态。
 */

#include "bsp_adc_current.h"

#include "bsp_config.h"

#define BSP_ADC_CURRENT_TIMEOUT_MS 10U
#define BSP_ADC_CURRENT_REF_MV     3300U
#define BSP_ADC_CURRENT_RAW_MAX    4095U

static ADC_HandleTypeDef s_hadc_current;

/**
 * @brief 将 HAL ADC 返回状态映射为 BSP 通用返回码。
 */
static BSP_Status_t BSP_ADC_Current_MapHalStatus(HAL_StatusTypeDef status)
{
  return (status == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/**
 * @brief 根据逻辑通道返回 STM32 ADC 通道号。
 */
static uint32_t BSP_ADC_Current_HalChannel(BSP_ADC_CurrentChannel_t channel)
{
  return (channel == BSP_ADC_CURRENT_BATTERY2) ? BSP_ADC_CURRENT2_CH : BSP_ADC_CURRENT1_CH;
}

BSP_Status_t BSP_ADC_Current_Init(void)
{
  s_hadc_current.Instance                   = BSP_ADC_CURRENT_INS;
  s_hadc_current.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  s_hadc_current.Init.Resolution            = ADC_RESOLUTION_12B;
  s_hadc_current.Init.ScanConvMode          = DISABLE;
  s_hadc_current.Init.ContinuousConvMode    = DISABLE;
  s_hadc_current.Init.DiscontinuousConvMode = DISABLE;
  s_hadc_current.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  s_hadc_current.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  s_hadc_current.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  s_hadc_current.Init.NbrOfConversion       = 1U;
  s_hadc_current.Init.DMAContinuousRequests = DISABLE;
  s_hadc_current.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  return (HAL_ADC_Init(&s_hadc_current) == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

BSP_Status_t BSP_ADC_Current_DeInit(void)
{
  if (s_hadc_current.Instance == 0) { return BSP_STATUS_OK; }
  return (HAL_ADC_DeInit(&s_hadc_current) == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

BSP_Status_t BSP_ADC_Current_Recover(void)
{
  (void)BSP_ADC_Current_DeInit();
  return BSP_ADC_Current_Init();
}

BSP_Status_t BSP_ADC_Current_ReadRaw(BSP_ADC_CurrentChannel_t channel, uint32_t *raw)
{
  ADC_ChannelConfTypeDef adc_channel;
  HAL_StatusTypeDef result;

  if (raw == 0) { return BSP_STATUS_ERROR; }
  if ((channel != BSP_ADC_CURRENT_BATTERY1) && (channel != BSP_ADC_CURRENT_BATTERY2)) { return BSP_STATUS_ERROR; }

  if (s_hadc_current.Instance == 0) {
    if (BSP_ADC_Current_Init() != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  }

  adc_channel.Channel      = BSP_ADC_Current_HalChannel(channel);
  adc_channel.Rank         = 1U;
  adc_channel.SamplingTime = BSP_ADC_CURRENT_SAMPLE_TIME;
  adc_channel.Offset       = 0U;
  if (HAL_ADC_ConfigChannel(&s_hadc_current, &adc_channel) != HAL_OK) { return BSP_STATUS_ERROR; }

  result = HAL_ADC_Start(&s_hadc_current);
  if (result != HAL_OK) { return BSP_ADC_Current_MapHalStatus(result); }

  result = HAL_ADC_PollForConversion(&s_hadc_current, BSP_ADC_CURRENT_TIMEOUT_MS);
  if (result == HAL_OK) { *raw = HAL_ADC_GetValue(&s_hadc_current); }

  (void)HAL_ADC_Stop(&s_hadc_current);
  return BSP_ADC_Current_MapHalStatus(result);
}

BSP_Status_t BSP_ADC_Current_ReadAverage(BSP_ADC_CurrentChannel_t channel, uint32_t *raw, uint8_t count)
{
  uint32_t sum = 0U;
  uint8_t i;

  if (raw == 0) { return BSP_STATUS_ERROR; }
  if (count == 0U) { count = 1U; }

  for (i = 0U; i < count; i++) {
    uint32_t sample;
    BSP_Status_t result = BSP_ADC_Current_ReadRaw(channel, &sample);
    if (result != BSP_STATUS_OK) { return result; }
    sum += sample;
  }

  *raw = (sum + ((uint32_t)count / 2U)) / (uint32_t)count;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv)
{
  uint32_t raw;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }
  if (BSP_ADC_Current_ReadAverage(channel, &raw, BSP_ADC_CURRENT_AVERAGE_COUNT) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  if (raw > BSP_ADC_CURRENT_RAW_MAX) { raw = BSP_ADC_CURRENT_RAW_MAX; }

  *voltage_mv = ((raw * BSP_ADC_CURRENT_REF_MV) + (BSP_ADC_CURRENT_RAW_MAX / 2U)) / BSP_ADC_CURRENT_RAW_MAX;
  return BSP_STATUS_OK;
}
