/**
 * @file sensor_power2.c
 * @brief 实现第二块电池电源检测强类型驱动。
 *
 * @details
 * 本文件从 BSP_ADC2 读取 PA4/ADC2_IN4 电压，使用 `sensor_power_config.h` 中的第二电池动力电池
 * 曲线，并复用通用校准和滤波配置。低电压事实经过滤波、连续确认和回差处理后上报，系统级
 * 告警和关断策略由上层仲裁。
 */

#include "sensor_power2.h"

#include "bsp_adc2.h"
#include "sensor_power_config.h"

#include <string.h>

/**
 * @brief 第二块电池电量曲线上的一个标定点。
 */
typedef struct {
  uint32_t voltage_mv; /**< 曲线点对应的电池电压，单位：mV。 */
  uint8_t percent;     /**< 曲线点对应的电量百分比，范围：0 到 100。 */
} Power2_PercentPoint_t;

static const Power2_PercentPoint_t s_percent_curve2[] = {
  {POWER2_PERCENT_TABLE_EMPTY_MV, 0U},
  {POWER2_PERCENT_TABLE_5_MV, 5U},
  {POWER2_PERCENT_TABLE_10_MV, 10U},
  {POWER2_PERCENT_TABLE_20_MV, 20U},
  {POWER2_PERCENT_TABLE_30_MV, 30U},
  {POWER2_PERCENT_TABLE_40_MV, 40U},
  {POWER2_PERCENT_TABLE_50_MV, 50U},
  {POWER2_PERCENT_TABLE_60_MV, 60U},
  {POWER2_PERCENT_TABLE_70_MV, 70U},
  {POWER2_PERCENT_TABLE_80_MV, 80U},
  {POWER2_PERCENT_TABLE_90_MV, 90U},
  {POWER2_PERCENT_TABLE_FULL_MV, 100U}
};

static Power_Snapshot_t s_snapshot2;
static uint32_t s_filtered_voltage_mv2;
static uint8_t s_pending_percent_count2;
static uint8_t s_pending_low_voltage_count2;
static uint8_t s_initialized2;
static uint8_t s_filter_valid2;
static volatile uint8_t s_reinit_request2;

/**
 * @brief 对 BSP_ADC2 输出的电池电压做二次校准。
 */
static uint32_t Power2_ApplyCalibration(uint32_t measured_mv)
{
  uint32_t scaled_mv;
  int32_t calibrated_mv;

#if POWER_CAL_GAIN_DEN == 0
#error "POWER_CAL_GAIN_DEN must not be zero"
#endif

  scaled_mv = ((measured_mv * POWER_CAL_GAIN_NUM) + (POWER_CAL_GAIN_DEN / 2U)) / POWER_CAL_GAIN_DEN;
  calibrated_mv = (int32_t)scaled_mv + (int32_t)POWER_CAL_OFFSET_MV;
#if POWER_LOAD_COMPENSATION_ENABLE
  calibrated_mv += (int32_t)POWER_LOAD_COMPENSATION_MV;
#endif

  return (calibrated_mv > 0) ? (uint32_t)calibrated_mv : 0U;
}

/**
 * @brief 将百分比约束到配置的显示步进。
 */
static uint8_t Power2_RoundToStep(uint32_t percent)
{
  percent = ((percent + (POWER2_PERCENT_STEP / 2U)) / POWER2_PERCENT_STEP) * POWER2_PERCENT_STEP;
  return (percent > 100U) ? 100U : (uint8_t)percent;
}

/**
 * @brief 根据电压毫伏值计算配置曲线上的电量百分比。
 */
static uint8_t Power2_CalcPercent(uint32_t voltage_mv)
{
  uint32_t i;
  uint32_t percent;

  if (voltage_mv <= s_percent_curve2[0].voltage_mv) { return s_percent_curve2[0].percent; }

  for (i = 1U; i < (uint32_t)(sizeof(s_percent_curve2) / sizeof(s_percent_curve2[0])); i++) {
    const Power2_PercentPoint_t *low = &s_percent_curve2[i - 1U];
    const Power2_PercentPoint_t *high = &s_percent_curve2[i];
    uint32_t range_mv;
    uint32_t range_pct;

    if (voltage_mv > high->voltage_mv) { continue; }

    range_mv = high->voltage_mv - low->voltage_mv;
    range_pct = (uint32_t)high->percent - (uint32_t)low->percent;
    if (range_mv == 0U) { return high->percent; }

    percent = (uint32_t)low->percent + ((((voltage_mv - low->voltage_mv) * range_pct) + (range_mv / 2U)) / range_mv);
    return Power2_RoundToStep(percent);
  }

  return s_percent_curve2[(sizeof(s_percent_curve2) / sizeof(s_percent_curve2[0])) - 1U].percent;
}

/**
 * @brief 对电压采样做一阶低通滤波。
 */
static uint32_t Power2_FilterVoltage(uint32_t voltage_mv)
{
  if (s_filter_valid2 == 0U) {
    s_filtered_voltage_mv2 = voltage_mv;
    s_filter_valid2        = 1U;
  } else {
    s_filtered_voltage_mv2 = ((s_filtered_voltage_mv2 * POWER_FILTER_OLD_WEIGHT) + voltage_mv + (POWER_FILTER_TOTAL / 2U)) / POWER_FILTER_TOTAL;
  }

  return s_filtered_voltage_mv2;
}

/**
 * @brief 计算指定电量百分比对应的标称电压。
 */
static uint32_t Power2_PercentCenterMv(uint8_t percent)
{
  uint32_t i;

  if (percent <= s_percent_curve2[0].percent) { return s_percent_curve2[0].voltage_mv; }

  for (i = 1U; i < (uint32_t)(sizeof(s_percent_curve2) / sizeof(s_percent_curve2[0])); i++) {
    const Power2_PercentPoint_t *low = &s_percent_curve2[i - 1U];
    const Power2_PercentPoint_t *high = &s_percent_curve2[i];
    uint32_t range_mv;
    uint32_t range_pct;

    if (percent > high->percent) { continue; }

    range_mv = high->voltage_mv - low->voltage_mv;
    range_pct = (uint32_t)high->percent - (uint32_t)low->percent;
    if (range_pct == 0U) { return high->voltage_mv; }

    return low->voltage_mv + ((((uint32_t)(percent - low->percent) * range_mv) + (range_pct / 2U)) / range_pct);
  }

  return s_percent_curve2[(sizeof(s_percent_curve2) / sizeof(s_percent_curve2[0])) - 1U].voltage_mv;
}

/**
 * @brief 判断滤波电压是否越过当前百分比档位的滞回边界。
 */
static uint8_t Power2_PercentPassesHysteresis(uint8_t previous, uint8_t candidate, uint32_t filtered_voltage_mv)
{
  uint32_t center_mv = Power2_PercentCenterMv(previous);

  if (candidate > previous) { return (filtered_voltage_mv >= (center_mv + POWER2_PERCENT_HYSTERESIS_MV)) ? 1U : 0U; }
  return ((filtered_voltage_mv + POWER2_PERCENT_HYSTERESIS_MV) <= center_mv) ? 1U : 0U;
}

/**
 * @brief 连续多次确认后才更新百分比档位，抑制临界点跳变。
 */
static uint8_t Power2_ApplyPercentConfirm(uint8_t previous, uint8_t candidate, uint32_t filtered_voltage_mv)
{
  if (candidate == previous) {
    s_pending_percent_count2 = 0U;
    return previous;
  }

  if (Power2_PercentPassesHysteresis(previous, candidate, filtered_voltage_mv) == 0U) {
    s_pending_percent_count2 = 0U;
    return previous;
  }

  if (s_pending_percent_count2 < POWER2_PERCENT_CONFIRM_COUNT) { s_pending_percent_count2++; }
  if (s_pending_percent_count2 >= POWER2_PERCENT_CONFIRM_COUNT) {
    s_pending_percent_count2 = 0U;
    return candidate;
  }

  return previous;
}

/**
 * @brief 根据滤波电压连续确认第二电池低压进入和解除，避免电机负载瞬态导致告警闪烁。
 */
static uint8_t Power2_ApplyLowVoltageConfirm(uint8_t previous, uint32_t filtered_voltage_mv)
{
#if POWER2_LOW_RECOVER_MV <= POWER2_LOW_WARNING_MV
#error "POWER2_LOW_RECOVER_MV must be greater than POWER2_LOW_WARNING_MV"
#endif

  if (previous == 0U) {
    if (filtered_voltage_mv >= POWER2_LOW_WARNING_MV) {
      s_pending_low_voltage_count2 = 0U;
      return previous;
    }
  } else {
    if (filtered_voltage_mv <= POWER2_LOW_RECOVER_MV) {
      s_pending_low_voltage_count2 = 0U;
      return previous;
    }
  }

  if (s_pending_low_voltage_count2 < POWER2_LOW_CONFIRM_COUNT) { s_pending_low_voltage_count2++; }
  if (s_pending_low_voltage_count2 >= POWER2_LOW_CONFIRM_COUNT) {
    s_pending_low_voltage_count2 = 0U;
    return (previous == 0U) ? 1U : 0U;
  }

  return previous;
}

Power_Result_t Sensor_Power2_Init(void)
{
  memset(&s_snapshot2, 0, sizeof(s_snapshot2));
  s_filtered_voltage_mv2       = 0U;
  s_pending_percent_count2     = 0U;
  s_pending_low_voltage_count2 = 0U;
  s_filter_valid2              = 0U;
  s_initialized2               = 1U;
  return POWER_RESULT_OK;
}

void Sensor_Power2_RequestReinit(void)
{
  s_reinit_request2 = 1U;
}

Power_Result_t Sensor_Power2_Service(uint32_t now_ms)
{
  uint32_t measured_voltage_mv = 0U;
  uint32_t voltage_mv;
  uint32_t filtered_voltage_mv;
  uint8_t candidate_percent;

  if (s_reinit_request2 != 0U) {
    s_reinit_request2 = 0U;
    (void)Sensor_Power2_Init();
  }
  if (s_initialized2 == 0U) { return POWER_RESULT_NO_DATA; }

  if (BSP_ADC2_ReadVoltageMv(&measured_voltage_mv) != BSP_STATUS_OK) {
    s_snapshot2.error_count++;
    return POWER_RESULT_IO_ERROR;
  }

  voltage_mv = Power2_ApplyCalibration(measured_voltage_mv);
  s_snapshot2.rx_sequence++;
  if (s_snapshot2.rx_sequence == 0U) { s_snapshot2.rx_sequence = 1U; }
  s_snapshot2.sample_time_ms = now_ms;
  s_snapshot2.voltage_v      = ((float)voltage_mv) / 1000.0f;
  filtered_voltage_mv        = Power2_FilterVoltage(voltage_mv);
  candidate_percent          = Power2_CalcPercent(filtered_voltage_mv);
  s_snapshot2.percent        = (s_snapshot2.rx_sequence == 1U) ? candidate_percent : Power2_ApplyPercentConfirm(s_snapshot2.percent, candidate_percent, filtered_voltage_mv);
  s_snapshot2.low_voltage    = Power2_ApplyLowVoltageConfirm(s_snapshot2.low_voltage, filtered_voltage_mv);

  return POWER_RESULT_OK;
}

Power_Result_t Sensor_Power2_CopySnapshot(Power_Snapshot_t *out)
{
  if (out == 0) { return POWER_RESULT_INVALID_PARAM; }

  *out = s_snapshot2;
  return (s_snapshot2.rx_sequence != 0U) ? POWER_RESULT_OK : POWER_RESULT_NO_DATA;
}

Power_Result_t Sensor_Power2_GetStatus(Power_Status_t *out)
{
  if (out == 0) { return POWER_RESULT_INVALID_PARAM; }

  out->rx_sequence    = s_snapshot2.rx_sequence;
  out->sample_time_ms = s_snapshot2.sample_time_ms;
  out->error_count    = s_snapshot2.error_count;
  out->percent        = s_snapshot2.percent;
  out->low_voltage    = s_snapshot2.low_voltage;
  out->reserved       = 0U;
  return (s_snapshot2.rx_sequence != 0U) ? POWER_RESULT_OK : POWER_RESULT_NO_DATA;
}
