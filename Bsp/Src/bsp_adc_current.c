/**
 * @file bsp_adc_current.c
 * @brief 实现 PC0/PC1 两路电流计模拟输出采样。
 * @details
 * 本文件属于 BSP 层，只负责 ADC3 初始化、通道切换、轮询采样、平均采样和 ADC 引脚
 * 电压换算。不得在本层解释电流计比例、零点或电池状态。
 */

#include "bsp_adc_current.h"

#include "bsp_adc.h" /* 复用 ADC1 的 VREFINT 实测 VDDA(VDDA 为 ADC1/ADC2/ADC3 公共供电) */
#include "bsp_config.h"

#define BSP_ADC_CURRENT_TIMEOUT_MS 10U
#define BSP_ADC_CURRENT_RAW_MAX    4095U

static ADC_HandleTypeDef s_hadc_current;

/** @brief 最近成功采样的诊断缓存；下标 0/1 对应 PC0/PC1，不触发额外采样。 */
static uint32_t s_diag_adc_mv[2];
static uint8_t  s_diag_floating[2];

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

/* 浮空判定阈值(原始码)：见 BSP_ADC_Current_PinFloating。约 300 码≈0.24V(pin)。 */
#define BSP_ADC_CURRENT_FLOAT_DELTA_RAW 300U

/* 浮空检测开关。诊断证明:电流计 CURR 输出对"预置拉高"的下沉能力弱，低电流(~0.2V)
   时会被浮空检测误判成高阻并清零(把真实小电流误杀)。传感器已实测接好，故关闭检测、
   直接读真实电压。若以后要重新识别"未接传感器"，需改进检测策略后再置 1。 */
#ifndef BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
#define BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE 0U
#endif

#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
/**
 * @brief 仅供显式启用的浮空诊断预置引脚电平，返回前恢复模拟输入。
 * @param[in] channel 已校验的电流通道。
 * @param[in] high 0 为低电平，非 0 为高电平。
 * @note 正常采样不得调用；推挽探测会扰动外部模拟输出，生产配置默认禁用。
 */
static void BSP_ADC_Current_PrechargePin(BSP_ADC_CurrentChannel_t channel, uint8_t high)
{
  GPIO_InitTypeDef gpio;
  GPIO_TypeDef    *port = (channel == BSP_ADC_CURRENT_BATTERY2) ? BSP_ADC_CURRENT2_PORT : BSP_ADC_CURRENT1_PORT;
  uint16_t         pin  = (channel == BSP_ADC_CURRENT_BATTERY2) ? BSP_ADC_CURRENT2_PIN : BSP_ADC_CURRENT1_PIN;
  volatile uint32_t d;

  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin   = pin;
  HAL_GPIO_Init(port, &gpio);
  HAL_GPIO_WritePin(port, pin, (high != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  for (d = 0U; d < 400U; ++d) { __NOP(); }

  gpio.Mode = GPIO_MODE_ANALOG;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &gpio);
}
#endif

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

/**
 * @brief 在模拟输入状态读取指定通道，不驱动外部电流传感器的输出引脚。
 * @param[in] channel 第一或第二电流通道。
 * @param[out] raw 成功时输出 12 位原始码，失败时保持原值。
 * @return 参数或转换失败返回 ERROR，否则返回 OK。
 */
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

#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
/** @brief 仅在浮空诊断中预置电平后采样，所有转换均在模拟输入状态进行。 */
static BSP_Status_t BSP_ADC_Current_ReadOnePrecharged(BSP_ADC_CurrentChannel_t channel, uint8_t high, uint32_t *raw)
{
  ADC_ChannelConfTypeDef adc_channel;
  HAL_StatusTypeDef result;

  if (s_hadc_current.Instance == 0) {
    if (BSP_ADC_Current_Init() != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  }

  adc_channel.Channel      = BSP_ADC_Current_HalChannel(channel);
  adc_channel.Rank         = 1U;
  adc_channel.SamplingTime = BSP_ADC_CURRENT_SAMPLE_TIME;
  adc_channel.Offset       = 0U;
  if (HAL_ADC_ConfigChannel(&s_hadc_current, &adc_channel) != HAL_OK) { return BSP_STATUS_ERROR; }

  BSP_ADC_Current_PrechargePin(channel, high);
  result = HAL_ADC_Start(&s_hadc_current);
  if (result != HAL_OK) { (void)HAL_ADC_Stop(&s_hadc_current); return BSP_STATUS_ERROR; }
  result = HAL_ADC_PollForConversion(&s_hadc_current, BSP_ADC_CURRENT_TIMEOUT_MS);
  if (result == HAL_OK) { *raw = HAL_ADC_GetValue(&s_hadc_current); }
  (void)HAL_ADC_Stop(&s_hadc_current);
  return (result == HAL_OK) ? BSP_STATUS_OK : BSP_STATUS_ERROR;
}

/** @brief 比较高低预置后的原始码；转换失败与浮空结果分别返回。 */
static BSP_Status_t BSP_ADC_Current_PinFloating(BSP_ADC_CurrentChannel_t channel, uint8_t *floating)
{
  uint32_t raw_lo = 0U;
  uint32_t raw_hi = 0U;

  if (BSP_ADC_Current_ReadOnePrecharged(channel, 0U, &raw_lo) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  if (BSP_ADC_Current_ReadOnePrecharged(channel, 1U, &raw_hi) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  *floating = ((raw_hi > raw_lo) && ((raw_hi - raw_lo) > BSP_ADC_CURRENT_FLOAT_DELTA_RAW)) ? 1U : 0U;
  return BSP_STATUS_OK;
}
#endif

/**
 * @brief 平均采样并换算引脚毫伏值；可选浮空探测的全部引脚操作受编译开关门控。
 * @param[in] channel 第一或第二电流通道。
 * @param[out] voltage_mv 成功时输出毫伏值，失败时保持原值。
 * @return 参数、诊断探测或正常转换失败返回 ERROR，否则返回 OK。
 */
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv)
{
  uint32_t raw = 0U;
  uint32_t vdda;
  uint32_t pin_mv;
  uint8_t  floating = 0U;
  uint8_t  idx = (channel == BSP_ADC_CURRENT_BATTERY2) ? 1U : 0U;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }
  if ((channel != BSP_ADC_CURRENT_BATTERY1) && (channel != BSP_ADC_CURRENT_BATTERY2)) { return BSP_STATUS_ERROR; }

#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  if (BSP_ADC_Current_PinFloating(channel, &floating) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
#endif

  vdda = BSP_ADC_GetVddaMv(); /* 用 VREFINT 实测 VDDA 代替写死的 3.3V，消除电机负载导致的 VDDA 下陷误差 */
  if (BSP_ADC_Current_ReadAverage(channel, &raw, BSP_ADC_CURRENT_AVERAGE_COUNT) != BSP_STATUS_OK) { return BSP_STATUS_ERROR; }
  if (raw > BSP_ADC_CURRENT_RAW_MAX) { raw = BSP_ADC_CURRENT_RAW_MAX; }

  pin_mv = ((raw * vdda) + (BSP_ADC_CURRENT_RAW_MAX / 2U)) / BSP_ADC_CURRENT_RAW_MAX;
  s_diag_adc_mv[idx]   = pin_mv;
  s_diag_floating[idx] = floating;

#if BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE
  *voltage_mv = (floating != 0U) ? 0U : pin_mv; /* 判浮空则对外清零 */
#else
  *voltage_mv = pin_mv; /* 浮空检测已关闭：CURR 为高阻源，直接用真实电压 */
#endif
  return BSP_STATUS_OK;
}

void BSP_ADC_Current_GetDiag(BSP_ADC_CurrentChannel_t channel, uint32_t *adc_mv, uint8_t *floating)
{
  uint8_t idx = (channel == BSP_ADC_CURRENT_BATTERY2) ? 1U : 0U;

  if (adc_mv != 0)   { *adc_mv = s_diag_adc_mv[idx]; }
  if (floating != 0) { *floating = s_diag_floating[idx]; }
}
