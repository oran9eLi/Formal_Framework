/**
 * @file sensor_power2.c
 * @brief 实现第二块电池电源检测强类型驱动。
 *
 * @details
 * 参照 sensor_power.c(电池1) 实现，从 BSP_ADC2(PA4/ADC2_IN4) 读取电压、
 * 换算电量百分比并缓存最近一次电源快照。第二块电池与第一块同规格，
 * 电量阈值与稳定处理逻辑一致。
 */

#include "sensor_power2.h"

#include "bsp_adc2.h"

#include <string.h>

#define POWER2_ZERO_MV               9900U
#define POWER2_STEP5_MV              10200U
#define POWER2_STEP10_MV             10500U
#define POWER2_FULL_MV               12550U
#define POWER2_FILTER_OLD_WEIGHT     3U
#define POWER2_FILTER_TOTAL          4U
#define POWER2_PERCENT_STEP          5U
#define POWER2_PERCENT_CONFIRM_COUNT 10U
#define POWER2_PERCENT_HYSTERESIS_MV 200U
#define POWER2_PRESENT_MV            5000U

static Power_Snapshot_t s_snapshot2;
static uint32_t s_filtered_voltage_mv2;
static uint8_t s_pending_percent_count2;
static uint8_t s_initialized2;
static uint8_t s_filter_valid2;
static uint8_t s_present2;
static volatile uint8_t s_reinit_request2;

/**
 * @brief 根据电压毫伏值计算 5% 步进的电量百分比。
 *
 * @param[in] voltage_mv 已滤波电压，单位：mV。
 *
 * @return 电量百分比，范围：0 到 100。
 */
static uint8_t Power2_CalcPercent(uint32_t voltage_mv)
{
  uint32_t range_mv;
  uint32_t percent;

  if (voltage_mv < POWER2_ZERO_MV) { return 0U; }
  if (voltage_mv < POWER2_STEP5_MV) { return 5U; }
  if (voltage_mv < POWER2_STEP10_MV) { return 10U; }
  if (voltage_mv >= POWER2_FULL_MV) { return 100U; }

  range_mv = POWER2_FULL_MV - POWER2_STEP10_MV;
  percent  = 10U + ((((voltage_mv - POWER2_STEP10_MV) * 90U) + (range_mv / 2U)) / range_mv);
  percent  = ((percent + (POWER2_PERCENT_STEP / 2U)) / POWER2_PERCENT_STEP) * POWER2_PERCENT_STEP;
  if (percent > 100U) { percent = 100U; }
  return (uint8_t)percent;
}

/**
 * @brief 对电压采样做一阶低通滤波。
 *
 * @param[in] voltage_mv 最新电压采样值，单位：mV。
 *
 * @return 滤波后的电压值，单位：mV。
 */
static uint32_t Power2_FilterVoltage(uint32_t voltage_mv)
{
  if (s_filter_valid2 == 0U) {
    s_filtered_voltage_mv2 = voltage_mv;
    s_filter_valid2        = 1U;
  } else {
    s_filtered_voltage_mv2 = ((s_filtered_voltage_mv2 * POWER2_FILTER_OLD_WEIGHT) + voltage_mv + (POWER2_FILTER_TOTAL / 2U)) / POWER2_FILTER_TOTAL;
  }

  return s_filtered_voltage_mv2;
}

/**
 * @brief 计算指定电量百分比对应的标称电压。
 *
 * @param[in] percent 电量百分比，范围：0 到 100。
 *
 * @return 百分比对应的标称电压，单位：mV。
 */
static uint32_t Power2_PercentCenterMv(uint8_t percent)
{
  uint32_t range_mv;
  uint32_t scaled_mv;

  if (percent == 0U) { return POWER2_ZERO_MV; }
  if (percent == 5U) { return (POWER2_ZERO_MV + POWER2_STEP5_MV) / 2U; }
  if (percent <= 10U) { return POWER2_STEP10_MV; }
  if (percent >= 100U) { return POWER2_FULL_MV; }

  range_mv  = POWER2_FULL_MV - POWER2_STEP10_MV;
  scaled_mv = (((uint32_t)(percent - 10U)) * range_mv) / 90U;
  return POWER2_STEP10_MV + scaled_mv;
}

/**
 * @brief 判断滤波电压是否已经越过当前发布百分比的滞回边界。
 *
 * @param[in] previous 当前已发布百分比。
 * @param[in] candidate 本次计算出的候选百分比。
 * @param[in] filtered_voltage_mv 已滤波电压，单位：mV。
 *
 * @return 1 表示允许进入连续确认流程，0 表示仍处于滞回区间。
 */
static uint8_t Power2_PercentPassesHysteresis(uint8_t previous, uint8_t candidate, uint32_t filtered_voltage_mv)
{
  uint32_t center_mv = Power2_PercentCenterMv(previous);

  if (candidate > previous) { return (filtered_voltage_mv >= (center_mv + POWER2_PERCENT_HYSTERESIS_MV)) ? 1U : 0U; }
  return ((filtered_voltage_mv + POWER2_PERCENT_HYSTERESIS_MV) <= center_mv) ? 1U : 0U;
}

/**
 * @brief 只有越过滞回边界且连续多次得到新候选百分比时才更新。
 *
 * @param[in] previous 当前已发布百分比。
 * @param[in] candidate 本次计算出的候选百分比。
 * @param[in] filtered_voltage_mv 已滤波电压，单位：mV。
 *
 * @return 应发布的电量百分比。
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

Power_Result_t Sensor_Power2_Init(void)
{
  memset(&s_snapshot2, 0, sizeof(s_snapshot2));
  s_filtered_voltage_mv2   = 0U;
  s_pending_percent_count2 = 0U;
  s_filter_valid2          = 0U;
  s_present2               = 0U;
  s_initialized2           = 1U;
  return POWER_RESULT_OK;
}

void Sensor_Power2_RequestReinit(void)
{
  s_reinit_request2 = 1U;
}

Power_Result_t Sensor_Power2_Service(uint32_t now_ms)
{
  uint32_t voltage_mv = 0U;
  uint32_t filtered_voltage_mv;
  uint8_t candidate_percent;
  uint8_t now_present;
  uint8_t fresh_insert;

  if (s_reinit_request2 != 0U) {
    s_reinit_request2 = 0U;
    (void)Sensor_Power2_Init();
  }
  if (s_initialized2 == 0U) { return POWER_RESULT_NO_DATA; }

  if (BSP_ADC2_ReadVoltageMv(&voltage_mv) != BSP_STATUS_OK) {
    s_snapshot2.error_count++;
    return POWER_RESULT_IO_ERROR;
  }

  /* 接入跳变检测：电压由"未接入"(< 接入门限)跳到"接入"视为热插拔。此时以本次
     读数(BSP 已做 8x 过采样)重新播种滤波并直接采用候选电量、跳过 10 次确认，
     使插上瞬间立即给出真实电量；其后恢复常规滤波/确认以抑制带载抖动。 */
  now_present  = (voltage_mv >= POWER2_PRESENT_MV) ? 1U : 0U;
  fresh_insert = ((now_present != 0U) && (s_present2 == 0U)) ? 1U : 0U;
  s_present2   = now_present;

  s_snapshot2.rx_sequence++;
  if (s_snapshot2.rx_sequence == 0U) { s_snapshot2.rx_sequence = 1U; }
  s_snapshot2.sample_time_ms = now_ms;

  if (fresh_insert != 0U) {
    s_filter_valid2          = 0U;
    s_pending_percent_count2 = 0U;
  }
  filtered_voltage_mv = Power2_FilterVoltage(voltage_mv);
  s_snapshot2.voltage_v = ((float)filtered_voltage_mv) / 1000.0f;
  candidate_percent   = Power2_CalcPercent(filtered_voltage_mv);
  s_snapshot2.percent = ((s_snapshot2.rx_sequence == 1U) || (fresh_insert != 0U)) ? candidate_percent : Power2_ApplyPercentConfirm(s_snapshot2.percent, candidate_percent, filtered_voltage_mv);
  s_snapshot2.low_voltage = (voltage_mv < POWER2_ZERO_MV) ? 1U : 0U;

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
