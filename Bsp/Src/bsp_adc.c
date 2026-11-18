/**
 * @file bsp_adc.c
 * @brief BSP 层 ADC1 内部参考与可选第一组外部电压采样。
 * @details 第一组停用时仅读取内部 VREFINT，供 ADC2/ADC3 修正 VDDA；不得触碰 PA5。
 */

#include "bsp_adc.h"

#include "bsp_config.h"
#include "stm32f4xx_hal.h"
#include "bsp_critical.h"
#include <string.h>

#define BSP_ADC_TIMEOUT_MS    10U
#define BSP_ADC_AVERAGE_COUNT 16U /* 电池通道平均次数(加重滤波) */
#define BSP_ADC_VREF_COUNT    8U  /* VREFINT 平均次数 */
#define BSP_ADC_REF_MV        3300U
#define BSP_ADC_RAW_MAX       4095U
#define BSP_ADC_SAMPLETIME    ADC_SAMPLETIME_480CYCLES /* 长采样：高阻分压充分建立 + VREFINT 要求 */

/* STM32F407 出厂校准的 VREFINT 原始值(VDDA=3.3V 下测得)，地址固定。
   实测 VDDA = 3300 * VREFINT_CAL / VREFINT_raw，用于消除 VDDA 纹波对读数的比例误差。 */
/* 主机验证可注入只读校准样本；正式构建默认使用 STM32F407 出厂校准区。 */
#ifndef BSP_ADC_VREFINT_CAL
#define BSP_ADC_VREFINT_CAL    (*((const volatile uint16_t *)0x1FFF7A2AU))
#endif
#define BSP_ADC_VREFINT_CAL_MV 3300U

/* 当前PCLK2/4为21MHz。DS8626规定VDDA<2.4V时ADC时钟最高18MHz，不能把该区间估值当本配置正常。 */
#ifndef BSP_ADC_VDDA_MIN_MV
#define BSP_ADC_VDDA_MIN_MV 2400U
#endif
#ifndef BSP_ADC_VDDA_MAX_MV
#define BSP_ADC_VDDA_MAX_MV 3600U
#endif
#if BSP_ADC_VDDA_MIN_MV == 0 || BSP_ADC_VDDA_MIN_MV > BSP_ADC_VDDA_MAX_MV
#error "Invalid ADC reference diagnostic voltage window"
#endif

static ADC_HandleTypeDef s_hadc;
static uint8_t s_initialized;
static BSP_ADC_ReferenceDiag_t s_reference_diag;
static uint8_t s_reference_raw_invalid;
static uint32_t s_reference_bad_raw;

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

/** @brief 初始化 ADC1；关闭第一组时默认选择内部参考，不初始化外部采集脚。 */
BSP_Status_t BSP_ADC_Init(void)
{
#if BSP_ENABLE_ADC
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

#if BSP_ENABLE_POWER1
  channel.Channel      = BSP_ADC_CH;
#else
  channel.Channel      = ADC_CHANNEL_VREFINT;
#endif
  channel.Rank         = 1U;
  channel.SamplingTime = BSP_ADC_SAMPLETIME;
  channel.Offset       = 0U;

  if (HAL_ADC_ConfigChannel(&s_hadc, &channel) != HAL_OK) { return BSP_STATUS_ERROR; }

  s_initialized = 1U;
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_ERROR;
#endif
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
#if BSP_ENABLE_POWER1
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
#endif

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
    uint32_t sample;
    /* 外部采集脚每次转换前拉低放电，消除浮空漂高；VREFINT 等内部通道不驱动。 */
#if BSP_ENABLE_POWER1
    if (channel == BSP_ADC_CH) { BSP_ADC_PrechargePin(0U); }
#endif
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
    sample = HAL_ADC_GetValue(&s_hadc);
    (void)HAL_ADC_Stop(&s_hadc);
    if (channel == ADC_CHANNEL_VREFINT && (sample == 0U || sample > BSP_ADC_RAW_MAX)) {
      s_reference_raw_invalid = 1U; s_reference_bad_raw = sample;
      return BSP_STATUS_ERROR;
    }
    sum += sample;
  }

  *raw = (sum + ((uint32_t)count / 2U)) / (uint32_t)count;
  return BSP_STATUS_OK;
}

/** @brief owner测量公共参考并发布尝试缓存；异常兼容回退3300，但明确保留失败原因与计算值。 */
uint32_t BSP_ADC_GetVddaMv(void)
{
  BSP_ADC_ReferenceDiag_t diag;
  uint32_t key;
  memset(&diag, 0, sizeof(diag));
  diag.sequence = s_reference_diag.sequence + 1U;
  diag.calibration_raw = BSP_ADC_VREFINT_CAL;
  diag.vdda_mv = BSP_ADC_REF_MV;
  diag.ready = 1U;
  s_reference_raw_invalid = 0U; s_reference_bad_raw = 0U;
  if (diag.calibration_raw == 0U) { diag.status = BSP_ADC_REF_FALLBACK_CAL_ZERO; }
  else if (diag.calibration_raw == UINT16_MAX) { diag.status = BSP_ADC_REF_FALLBACK_CAL_ERASED; }
  else if (diag.calibration_raw > BSP_ADC_RAW_MAX) { diag.status = BSP_ADC_REF_FALLBACK_CAL_RANGE; }
  else if (BSP_ADC_ReadChannelAverage(ADC_CHANNEL_VREFINT, BSP_ADC_SAMPLETIME, BSP_ADC_VREF_COUNT, &diag.vref_raw) != BSP_STATUS_OK) {
    diag.status = s_reference_raw_invalid ? BSP_ADC_REF_FALLBACK_RAW : BSP_ADC_REF_FALLBACK_READ;
    if (s_reference_raw_invalid) { diag.vref_raw = s_reference_bad_raw; }
  } else if (diag.vref_raw == 0U || diag.vref_raw > BSP_ADC_RAW_MAX) { diag.status = BSP_ADC_REF_FALLBACK_RAW; }
  else {
    diag.calculated_vdda_mv = (BSP_ADC_VREFINT_CAL_MV * (uint32_t)diag.calibration_raw) / diag.vref_raw;
    if (diag.calculated_vdda_mv < BSP_ADC_VDDA_MIN_MV || diag.calculated_vdda_mv > BSP_ADC_VDDA_MAX_MV) {
      diag.status = BSP_ADC_REF_FALLBACK_VDDA_RANGE;
    } else { diag.vdda_mv = diag.calculated_vdda_mv; diag.status = BSP_ADC_REF_MEASURED; }
  }
  key = BSP_Critical_Enter(); s_reference_diag = diag; BSP_Critical_Exit(key);
  return diag.vdda_mv;
}

/** @brief 只复制owner参考缓存，失败不写输出；不触碰ADC、工厂区或通道配置。 */
BSP_Status_t BSP_ADC_GetReferenceDiag(BSP_ADC_ReferenceDiag_t *out)
{
  uint32_t key;
  if (out == 0) { return BSP_STATUS_ERROR; }
  key = BSP_Critical_Enter();
  if (s_reference_diag.ready == 0U) { BSP_Critical_Exit(key); return BSP_STATUS_ERROR; }
  *out = s_reference_diag;
  BSP_Critical_Exit(key);
  return BSP_STATUS_OK;
}

BSP_Status_t BSP_ADC_ReadRaw(uint32_t *raw)
{
#if BSP_ENABLE_POWER1
  return BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, 1U, raw);
#else
  (void)raw;
  return BSP_STATUS_ERROR;
#endif
}

BSP_Status_t BSP_ADC_ReadAverage(uint32_t *raw, uint8_t count)
{
#if BSP_ENABLE_POWER1
  return BSP_ADC_ReadChannelAverage(BSP_ADC_CH, BSP_ADC_SAMPLETIME, count, raw);
#else
  (void)raw;
  (void)count;
  return BSP_STATUS_ERROR;
#endif
}

/* 单次转换：先把 PA5 预置到 high 指定电平再采样。用于浮空检测。 */
#if BSP_ENABLE_POWER1
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
#endif

/** @brief 读取第一组电压；停用时返回错误且不改写输出、不执行外部采样。 */
BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
#if BSP_ENABLE_POWER1
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
#else
  (void)voltage_mv;
  return BSP_STATUS_ERROR;
#endif
}
