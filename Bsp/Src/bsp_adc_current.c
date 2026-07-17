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

/* 临时诊断缓存：ReadVoltageMv 每次采样后记录每路原始引脚电压与浮空标志，供调试打印。
   下标 0=BATTERY1(PC0)，1=BATTERY2(PC1)。定位电流计接线后可连同 GetDiag 一起移除。 */
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
#define BSP_ADC_CURRENT_FLOAT_CHECK_ENABLE 0U

/* 采样前把电流计模拟脚预置到指定电平：high=0 拉低、high=1 拉高，数 µs 后切回模拟态。
   拉低=软件放电减小浮空漂高；配合拉高用于浮空检测(见 PinFloating)。 */
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

  /* 转换前拉低放电，消除未接电流计时的浮空漂高。 */
  BSP_ADC_Current_PrechargePin(channel, 0U);

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

/* 单次转换：先把该通道脚预置到 high 电平再采样，用于浮空检测。 */
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

/* 浮空(未接电流计)检测：预置低采一次、预置高采一次，两读相差过大即判浮空。
   真实低阻源两次都被拉回真值 → 差≈噪声；浮空脚跟随预置 → 差巨大。 */
static uint8_t BSP_ADC_Current_PinFloating(BSP_ADC_CurrentChannel_t channel)
{
  uint32_t raw_lo = 0U;
  uint32_t raw_hi = 0U;

  if (BSP_ADC_Current_ReadOnePrecharged(channel, 0U, &raw_lo) != BSP_STATUS_OK) { return 0U; }
  if (BSP_ADC_Current_ReadOnePrecharged(channel, 1U, &raw_hi) != BSP_STATUS_OK) { return 0U; }
  return ((raw_hi > raw_lo) && ((raw_hi - raw_lo) > BSP_ADC_CURRENT_FLOAT_DELTA_RAW)) ? 1U : 0U;
}

BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv)
{
  uint32_t raw = 0U;
  uint32_t vdda;
  uint32_t pin_mv;
  uint8_t  floating;
  uint8_t  idx = (channel == BSP_ADC_CURRENT_BATTERY2) ? 1U : 0U;

  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }

  /* 先做浮空判定，仅用于诊断标志；再无条件采一次真实引脚电压。 */
  floating = BSP_ADC_Current_PinFloating(channel);

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
