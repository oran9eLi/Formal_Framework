/**
 * @file bsp_adc.c
 * @brief 实现用于电源采样的板级 ADC 接口。
 */

#include "bsp_adc.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"

#define BSP_ADC_TIMEOUT_MS    10U
#define BSP_ADC_AVERAGE_COUNT 16U /* 电池通道平均次数(加重滤波) */
#define BSP_ADC_VREF_COUNT    8U  /* VREFINT 平均次数 */
#define BSP_ADC_REF_MV        3300U
#define BSP_ADC_RAW_MAX       4095U
#define BSP_ADC_SAMPLETIME    ADC_SAMPLETIME_480CYCLES /* 长采样：高阻分压充分建立 + VREFINT 要求 */

/* STM32F407 出厂校准的 VREFINT 原始值(VDDA=3.3V 下测得)，地址固定。
   实测 VDDA = 3300 * VREFINT_CAL / VREFINT_raw，用于消除 VDDA 纹波对读数的比例误差。 */
#define BSP_ADC_VREFINT_CAL    (*((const volatile uint16_t *)0x1FFF7A2AU))
#define BSP_ADC_VREFINT_CAL_MV 3300U

static ADC_HandleTypeDef s_hadc;
static uint8_t s_initialized;

/**
 * @brief 将 HAL ADC 返回状态映射为 BSP 通用返回码。
 *
 * @param[in] status HAL 返回状态。
 *
 * @return BSP 通用返回码。
 */
static BSP_Status_t BSP_ADC_MapHalStatus(HAL_StatusTypeDef status)
{
  if (status == HAL_OK) { return BSP_STATUS_OK; }
  if (status == HAL_BUSY) { return BSP_STATUS_BUSY; }
  if (status == HAL_TIMEOUT) { return BSP_STATUS_TIMEOUT; }
  return BSP_STATUS_ERROR;
}

BSP_Status_t BSP_ADC_Init(void)
{
  ADC_ChannelConfTypeDef channel;

  if (s_initialized != 0U) { return BSP_STATUS_OK; }

  s_hadc.Instance                   = BSP_ADC_INS;
  s_hadc.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV4;
  s_hadc.Init.Resolution            = ADC_RESOLUTION_12B;
  s_hadc.Init.ScanConvMode          = DISABLE;
  s_hadc.Init.ContinuousConvMode    = DISABLE;
  s_hadc.Init.DiscontinuousConvMode = DISABLE;
  s_hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
  s_hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
  s_hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
  s_hadc.Init.NbrOfConversion       = 1U;
  s_hadc.Init.DMAContinuousRequests = DISABLE;
  s_hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

  if (HAL_ADC_Init(&s_hadc) != HAL_OK) { return BSP_STATUS_ERROR; }

  channel.Channel      = BSP_ADC_CH;
  channel.Rank         = 1U;
  channel.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  channel.Offset       = 0U;

  if (HAL_ADC_ConfigChannel(&s_hadc, &channel) != HAL_OK) { return BSP_STATUS_ERROR; }

  s_initialized = 1U;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_DeInit(void)
{
  if (s_initialized == 0U) { return BSP_STATUS_OK; }

  if (HAL_ADC_DeInit(&s_hadc) != HAL_OK) { return BSP_STATUS_ERROR; }

  s_initialized = 0U;
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_Recover(void)
{
  (void)BSP_ADC_DeInit();
  return BSP_ADC_Init();
}

/* 配置指定通道与采样时间，连续转换 count 次并返回四舍五入平均原始值。
   每次调用都重配通道，便于在电池通道与内部 VREFINT 通道之间切换。 */
static BSP_Status_t BSP_ADC_ReadChannelAverage(uint32_t channel, uint32_t sampletime, uint8_t count, uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg;
  uint32_t sum = 0U;
  uint8_t i;
  HAL_StatusTypeDef hs;

  if ((raw == 0) || (count == 0U)) { return BSP_STATUS_ERROR; }

  BSP_ADC_Init();
  if (s_initialized == 0U) { return BSP_STATUS_ERROR; }

  cfg.Channel      = channel;
  cfg.Rank         = 1U;
  cfg.SamplingTime = sampletime;
  cfg.Offset       = 0U;
  if (HAL_ADC_ConfigChannel(&s_hadc, &cfg) != HAL_OK) { return BSP_STATUS_ERROR; }

  for (i = 0U; i < count; ++i) {
    hs = HAL_ADC_Start(&s_hadc);
    if (hs != HAL_OK) {
      (void)HAL_ADC_Stop(&s_hadc);
      return BSP_ADC_MapHalStatus(hs);
    }
    hs = HAL_ADC_PollForConversion(&s_hadc, BSP_ADC_TIMEOUT_MS);
    if (hs != HAL_OK) {
      (void)HAL_ADC_Stop(&s_hadc);
      return BSP_ADC_MapHalStatus(hs);
    }
    sum += HAL_ADC_GetValue(&s_hadc);
    (void)HAL_ADC_Stop(&s_hadc);
  }

  *raw = (sum + ((uint32_t)count / 2U)) / (uint32_t)count;
  return BSP_STATUS_OK;
}

uint32_t BSP_ADC_GetVddaMv(void)
{
  uint32_t vref_raw = 0U;
  uint16_t cal      = BSP_ADC_VREFINT_CAL;

  /* 校准值无效或读取失败时退回标称 3.3V，不影响功能。 */
  if (cal == 0U) { return BSP_ADC_REF_MV; }
  if (BSP_ADC_ReadChannelAverage(ADC_CHANNEL_VREFINT, BSP_ADC_SAMPLETIME, BSP_ADC_VREF_COUNT, &vref_raw) != BSP_STATUS_OK) { return BSP_ADC_REF_MV; }
  if (vref_raw == 0U) { return BSP_ADC_REF_MV; }

  return (BSP_ADC_VREFINT_CAL_MV * (uint32_t)cal) / vref_raw;
}

BSP_Status_t BSP_ADC_ReadRaw(uint32_t *raw)
{
  return BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, 1U, raw);
}

BSP_Status_t BSP_ADC_ReadAverage(uint32_t *raw, uint8_t count)
{
  return BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, count, raw);
}

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
  uint32_t raw = 0U;
  uint32_t vdda;
  uint32_t pin_mv;
  BSP_Status_t result;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }

  vdda   = BSP_ADC_GetVddaMv(); /* 用 VREFINT 实测 VDDA 代替写死的 3.3V */
  result = BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, BSP_ADC_AVERAGE_COUNT, &raw);
  if (result != BSP_STATUS_OK) { return result; }

  if (raw > BSP_ADC_RAW_MAX) { raw = BSP_ADC_RAW_MAX; }

  pin_mv      = ((raw * vdda) + (BSP_ADC_RAW_MAX / 2U)) / BSP_ADC_RAW_MAX;
  *voltage_mv = (pin_mv * BSP_ADC_DIVIDER_NUM) / BSP_ADC_DIVIDER_DEN;
  return BSP_STATUS_OK;
}
