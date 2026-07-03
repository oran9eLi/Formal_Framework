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

/* 浮空判定阈值(原始码)：先把 PA5 预置低采一次、再预置高采一次，真实低阻分压(约 9k)会
   把两次都拉回同一真值 → 两读之差≈噪声；未接电池时 PA5 浮空，会各自停在预置的低/高电位
   → 两读相差巨大。差值超过本阈值即判为浮空未接。约 300 码≈0.24V(pin)。 */
#define BSP_ADC_FLOAT_DELTA_RAW 300U

/* 采样前把外部采集脚预置到指定电平：high=0 拉低、high=1 拉高，数 µs 后切回模拟态。
   用于两个目的：(1) 拉低=软件放电，减小浮空漂高；(2) 配合拉高做浮空检测(见下)。
   仅用于外部引脚通道；内部 VREFINT 通道不可驱动。 */
static void BSP_ADC_PrechargePin(uint8_t high)
{
  GPIO_InitTypeDef gpio;
  volatile uint32_t d;

  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin   = BSP_ADC_PIN;
  HAL_GPIO_Init(BSP_ADC_PORT, &gpio);
  HAL_GPIO_WritePin(BSP_ADC_PORT, BSP_ADC_PIN, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  for (d = 0U; d < 400U; ++d) { __NOP(); }

  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BSP_ADC_PORT, &gpio);
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
    /* 外部采集脚每次转换前拉低放电，消除浮空漂高；VREFINT 等内部通道不驱动。 */
    if (channel == BSP_ADC_CH) { BSP_ADC_PrechargePin(0U); }
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

/* 单次转换：先把 PA5 预置到 high 指定电平再采样。用于浮空检测。 */
static BSP_Status_t BSP_ADC_ReadOnePrecharged(uint8_t high, uint32_t *raw)
{
  ADC_ChannelConfTypeDef cfg;
  HAL_StatusTypeDef hs;

  BSP_ADC_Init();
  if (s_initialized == 0U) { return BSP_STATUS_ERROR; }

  cfg.Channel      = BSP_ADC_CH;
  cfg.Rank         = 1U;
  cfg.SamplingTime = BSP_ADC_SAMPLETIME;
  cfg.Offset       = 0U;
  if (HAL_ADC_ConfigChannel(&s_hadc, &cfg) != HAL_OK) { return BSP_STATUS_ERROR; }

  BSP_ADC_PrechargePin(high);
  hs = HAL_ADC_Start(&s_hadc);
  if (hs != HAL_OK) { (void)HAL_ADC_Stop(&s_hadc); return BSP_ADC_MapHalStatus(hs); }
  hs = HAL_ADC_PollForConversion(&s_hadc, BSP_ADC_TIMEOUT_MS);
  if (hs != HAL_OK) { (void)HAL_ADC_Stop(&s_hadc); return BSP_ADC_MapHalStatus(hs); }
  *raw = HAL_ADC_GetValue(&s_hadc);
  (void)HAL_ADC_Stop(&s_hadc);
  return BSP_STATUS_OK;
}

/* 浮空(未接电池)检测：预置低采一次、预置高采一次，两读相差过大即判浮空。
   真实低阻源两次都被拉回真值 → 差≈噪声；浮空脚跟随预置 → 差巨大。 */
static uint8_t BSP_ADC_PinFloating(void)
{
  uint32_t raw_lo = 0U;
  uint32_t raw_hi = 0U;

  if (BSP_ADC_ReadOnePrecharged(0U, &raw_lo) != BSP_STATUS_OK) { return 0U; }
  if (BSP_ADC_ReadOnePrecharged(1U, &raw_hi) != BSP_STATUS_OK) { return 0U; }
  return ((raw_hi > raw_lo) && ((raw_hi - raw_lo) > BSP_ADC_FLOAT_DELTA_RAW)) ? 1U : 0U;
}

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
  uint32_t raw = 0U;
  uint32_t vdda;
  uint32_t pin_mv;
  BSP_Status_t result;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }

  /* 未接电池(采集脚浮空)时直接判 0，不让漏电流/干扰读出假电压。 */
  if (BSP_ADC_PinFloating() != 0U) { *voltage_mv = 0U; return BSP_STATUS_OK; }

  vdda   = BSP_ADC_GetVddaMv(); /* 用 VREFINT 实测 VDDA 代替写死的 3.3V */
  result = BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, BSP_ADC_AVERAGE_COUNT, &raw);
  if (result != BSP_STATUS_OK) { return result; }

  if (raw > BSP_ADC_RAW_MAX) { raw = BSP_ADC_RAW_MAX; }

  pin_mv      = ((raw * vdda) + (BSP_ADC_RAW_MAX / 2U)) / BSP_ADC_RAW_MAX;
  *voltage_mv = (pin_mv * BSP_ADC_DIVIDER_NUM) / BSP_ADC_DIVIDER_DEN;
  return BSP_STATUS_OK;
}
