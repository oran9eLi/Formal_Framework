/**
 * @file bsp_adc2.c
 * @brief 实现第二块电池 PA4/ADC2_IN4 的板级采样接口。
 *
 * @details
 * 本文件属于 BSP 层，只负责 ADC2 初始化、轮询采样、平均采样和按分压系数换算为输入电压。
 * 电量百分比、低电压事实和业务策略由 Sensor/Framework/Business 层处理。
 */

#include "bsp_adc2.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

#define BSP_ADC2_TIMEOUT_MS 10U
#define BSP_ADC2_REF_MV     3300U
#define BSP_ADC2_RAW_MAX    4095U

static ADC_HandleTypeDef s_hadc2;
static uint8_t s_initialized2;

/**
 * @brief 将 HAL ADC 返回状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码。
 */
static BSP_Status_t BSP_ADC2_MapHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) { return BSP_STATUS_OK; }
  if (status == HAL_BUSY) { return BSP_STATUS_BUSY; }
  if (status == HAL_TIMEOUT) { return BSP_STATUS_TIMEOUT; }
  return BSP_STATUS_ERROR;
}

BSP_Status_t BSP_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef channel;

  if (s_initialized2 != 0U) { return BSP_STATUS_OK; }

  s_hadc2.Instance                   = BSP_ADC2_INS;
  s_hadc2.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  s_hadc2.Init.Resolution            = ADC_RESOLUTION_12B;
  s_hadc2.Init.ScanConvMode          = DISABLE;
  s_hadc2.Init.ContinuousConvMode    = DISABLE;
  s_hadc2.Init.DiscontinuousConvMode = DISABLE;
  s_hadc2.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  s_hadc2.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  s_hadc2.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  s_hadc2.Init.NbrOfConversion       = 1U;
  s_hadc2.Init.DMAContinuousRequests = DISABLE;
  s_hadc2.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&s_hadc2) != HAL_OK) { return BSP_STATUS_ERROR; }

  channel.Channel      = BSP_ADC2_CH;
  channel.Rank         = 1U;
  channel.SamplingTime = BSP_ADC2_SAMPLE_TIME;
  channel.Offset       = 0U;

  if (HAL_ADC_ConfigChannel(&s_hadc2, &channel) != HAL_OK) {
    (void)HAL_ADC_DeInit(&s_hadc2);
    return BSP_STATUS_ERROR;
  }

  s_initialized2 = 1U;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC2_DeInit(void)
{
  if (s_initialized2 == 0U) { return BSP_STATUS_OK; }

  if (HAL_ADC_DeInit(&s_hadc2) != HAL_OK) { return BSP_STATUS_ERROR; }

  s_initialized2 = 0U;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC2_Recover(void)
{
  (void)BSP_ADC2_DeInit();
  return BSP_ADC2_Init();
}

BSP_Status_t BSP_ADC2_ReadRaw(uint32_t *raw)
{
  HAL_StatusTypeDef result;

  if (raw == 0) { return BSP_STATUS_ERROR; }

  BSP_ADC2_Init();
  if (s_initialized2 == 0U) { return BSP_STATUS_ERROR; }

  result = HAL_ADC_Start(&s_hadc2);
  if (result != HAL_OK) { return BSP_ADC2_MapHalStatus(result); }

  result = HAL_ADC_PollForConversion(&s_hadc2, BSP_ADC2_TIMEOUT_MS);
  if (result == HAL_OK) { *raw = HAL_ADC_GetValue(&s_hadc2); }

  (void)HAL_ADC_Stop(&s_hadc2);
  return BSP_ADC2_MapHalStatus(result);
}

BSP_Status_t BSP_ADC2_ReadAverage(uint32_t *raw, uint8_t count)
{
  uint32_t sum = 0U;
  uint8_t i;

  if ((raw == 0) || (count == 0U)) { return BSP_STATUS_ERROR; }

  for (i = 0U; i < count; ++i) {
    uint32_t sample     = 0U;
    BSP_Status_t result = BSP_ADC2_ReadRaw(&sample);

    if (result != BSP_STATUS_OK) { return result; }
    sum += sample;
  }

  *raw = (sum + ((uint32_t)count / 2U)) / (uint32_t)count;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *voltage_mv)
{
  uint32_t raw = 0U;
  uint32_t pin_mv;
  BSP_Status_t result;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }

  result = BSP_ADC2_ReadAverage(&raw, BSP_ADC2_AVERAGE_COUNT);
  if (result != BSP_STATUS_OK) { return result; }

  if (raw > BSP_ADC2_RAW_MAX) { raw = BSP_ADC2_RAW_MAX; }

  pin_mv      = ((raw * BSP_ADC2_REF_MV) + (BSP_ADC2_RAW_MAX / 2U)) / BSP_ADC2_RAW_MAX;
  *voltage_mv = (pin_mv * BSP_ADC2_DIVIDER_NUM) / BSP_ADC2_DIVIDER_DEN;
  return BSP_STATUS_OK;
}
