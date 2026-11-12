/**
 * @file bsp_adc2.c
 * @brief BSP 层第二组外部电压采样；当前测量整机主回路输入。
 *
 * @details 参照 bsp_adc.c 实现，使用独立的 ADC2 外设(PA4/ADC2_IN4)。
 */

#include "bsp_adc2.h"

#include "bsp_adc.h" /* 复用 ADC1 的 VREFINT 实测 VDDA(VDDA 为公共供电) */
#include "bsp_critical.h"
#include <string.h>
#include "bsp_config.h"
#include "stm32f4xx_hal.h"

#define BSP_ADC2_TIMEOUT_MS    10U
#define BSP_ADC2_AVERAGE_COUNT 16U /* 加重滤波 */
#define BSP_ADC2_REF_MV        3300U
#define BSP_ADC2_RAW_MAX       4095U
#define BSP_ADC2_SAMPLETIME    ADC_SAMPLETIME_480CYCLES /* 长采样：高阻分压充分建立 */

static ADC_HandleTypeDef s_hadc2;
static uint8_t s_initialized2;
static BSP_ADC2_Diag_t s_adc2_diag;

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

/** @brief 初始化第二组 ADC2；关闭该组时拒绝访问硬件。 */
BSP_Status_t BSP_ADC2_Init(void)
{
#if BSP_ENABLE_POWER2
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
  channel.SamplingTime = BSP_ADC2_SAMPLETIME;
  channel.Offset       = 0U;

  if (HAL_ADC_ConfigChannel(&s_hadc2, &channel) != HAL_OK) { return BSP_STATUS_ERROR; }

  s_initialized2 = 1U;
  return BSP_STATUS_OK;
#else
  return BSP_STATUS_ERROR;
#endif
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

/** @brief 读取第二组单次原始码；停用或失败时不改写输出。 */
BSP_Status_t BSP_ADC2_ReadRaw(uint32_t *raw)
{
  HAL_StatusTypeDef result;

  if ((raw == 0) || (BSP_ENABLE_POWER2 == 0U)) { return BSP_STATUS_ERROR; }

  BSP_ADC2_Init();
  if (s_initialized2 == 0U) { return BSP_STATUS_ERROR; }

  result = HAL_ADC_Start(&s_hadc2);
  if (result != HAL_OK) { return BSP_ADC2_MapHalStatus(result); }

  result = HAL_ADC_PollForConversion(&s_hadc2, BSP_ADC2_TIMEOUT_MS);
  if (result == HAL_OK) { *raw = HAL_ADC_GetValue(&s_hadc2); }

  (void)HAL_ADC_Stop(&s_hadc2);
  return BSP_ADC2_MapHalStatus(result);
}

/** @brief 在同一批转换中求平均与最值；不因诊断新增采样，失败不写输出。 */
static BSP_Status_t BSP_ADC2_ReadStatistics(uint32_t *raw, uint8_t count, uint32_t *minimum, uint32_t *maximum)
{
  uint64_t sum = 0U;
  uint32_t low = UINT32_MAX, high = 0U;
  uint8_t i;

  if ((raw == 0) || (count == 0U)) { return BSP_STATUS_ERROR; }

  for (i = 0U; i < count; ++i) {
    uint32_t sample     = 0U;
    BSP_Status_t result = BSP_ADC2_ReadRaw(&sample);

    if (result != BSP_STATUS_OK) { return result; }
    sum += sample;
    if (sample < low) { low = sample; }
    if (sample > high) { high = sample; }
  }

  *raw = (uint32_t)((sum + ((uint32_t)count / 2U)) / (uint32_t)count);
  if (minimum != 0) { *minimum = low; }
  if (maximum != 0) { *maximum = high; }
  return BSP_STATUS_OK;
}

/** @brief 保留原平均读取接口，由同一统计实现执行所请求的count次转换。 */
BSP_Status_t BSP_ADC2_ReadAverage(uint32_t *raw, uint8_t count)
{ return BSP_ADC2_ReadStatistics(raw, count, 0, 0); }

/** @brief 读取主回路输入毫伏值；复用 ADC1 的公共 VDDA 修正，停用时输出不变。 */
BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *voltage_mv)
{
  BSP_ADC2_Diag_t diag;
  uint32_t key;
  BSP_Status_t result;

  if ((voltage_mv == 0) || (BSP_ENABLE_POWER2 == 0U)) { return BSP_STATUS_ERROR; }
  memset(&diag, 0, sizeof(diag)); diag.sequence = s_adc2_diag.sequence + 1U; diag.ready = 1U;
  (void)BSP_ADC_GetVddaMv();
  result = BSP_ADC_GetReferenceDiag(&diag.reference);
  if (result == BSP_STATUS_OK) {
    result = BSP_ADC2_ReadStatistics(&diag.raw_average, BSP_ADC2_AVERAGE_COUNT, &diag.raw_min, &diag.raw_max);
  }
  if (result == BSP_STATUS_OK && (diag.raw_average > BSP_ADC2_RAW_MAX || diag.raw_max > BSP_ADC2_RAW_MAX)) { result = BSP_STATUS_ERROR; }
  if (result == BSP_STATUS_OK) {
    diag.pin_mv = ((diag.raw_average * diag.reference.vdda_mv) + (BSP_ADC2_RAW_MAX / 2U)) / BSP_ADC2_RAW_MAX;
    diag.input_mv = (diag.pin_mv * BSP_ADC2_DIVIDER_NUM) / BSP_ADC2_DIVIDER_DEN;
    diag.sample_valid = 1U;
  }
  diag.sample_status = (uint8_t)result;
  key = BSP_Critical_Enter(); s_adc2_diag = diag; BSP_Critical_Exit(key);
  if (result == BSP_STATUS_OK) { *voltage_mv = diag.input_mv; }
  return result;
}

/** @brief 只复制同轮电压/参考缓存，不发起ADC转换，warm初始化/恢复不擦历史证据。 */
BSP_Status_t BSP_ADC2_GetDiag(BSP_ADC2_Diag_t *out)
{
  uint32_t key;
  if (out == 0) { return BSP_STATUS_ERROR; }
  key = BSP_Critical_Enter();
  if (s_adc2_diag.ready == 0U) { BSP_Critical_Exit(key); return BSP_STATUS_ERROR; }
  *out = s_adc2_diag;
  BSP_Critical_Exit(key);
  return BSP_STATUS_OK;
}
