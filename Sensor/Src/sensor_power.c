/**
 * @file sensor_power.c
 * @brief 实现板载电源/电池检测强类型驱动。
 *
 * @details
 * 本文件负责从 BSP ADC 读取电压、换算电量百分比并缓存最近一次电源快照。
 * 低电压事实只表示电源采样结果，系统级告警和关机策略由上层仲裁。
 */

#include "sensor_power.h"

#include "bsp_adc.h"

#include <string.h>

/* 电量按电压线性映射(3S 锂电，9.0~12.6V)，5% 步进：9.0V=0%，12.6V=100%。 */
#define POWER_EMPTY_MV              9000U  /* 0%，下限 */
#define POWER_FULL_MV              12600U /* 100% */
#define POWER_FILTER_OLD_WEIGHT     3U
#define POWER_FILTER_TOTAL          4U
#define POWER_PERCENT_STEP          5U
#define POWER_PERCENT_CONFIRM_COUNT 10U
#define POWER_PERCENT_HYSTERESIS_MV 200U
#define POWER_PRESENT_MV            5000U

static Power_Snapshot_t s_snapshot;
static uint32_t s_filtered_voltage_mv;
static uint8_t s_pending_percent_count;
static uint8_t s_initialized;
static uint8_t s_filter_valid;
static uint8_t s_present;
static volatile uint8_t s_reinit_request;

/**
 * @brief 根据电压毫伏值计算 5% 步进的电量百分比。
 *
 * @param[in] voltage_mv 已滤波电压，单位：mV。
 *
 * @return 电量百分比，范围：0 到 100。
 */
static uint8_t Power_CalcPercent(uint32_t voltage_mv)
{
  uint32_t percent;

  if (voltage_mv <= POWER_EMPTY_MV) { return 0U; }
  if (voltage_mv >= POWER_FULL_MV) { return 100U; }

  percent = (((voltage_mv - POWER_EMPTY_MV) * 100U) / (POWER_FULL_MV - POWER_EMPTY_MV));
  percent = ((percent + (POWER_PERCENT_STEP / 2U)) / POWER_PERCENT_STEP) * POWER_PERCENT_STEP;
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
static uint32_t Power_FilterVoltage(uint32_t voltage_mv)
{
  if (s_filter_valid == 0U) {
    s_filtered_voltage_mv = voltage_mv;
    s_filter_valid        = 1U;
  } else {
    s_filtered_voltage_mv = ((s_filtered_voltage_mv * POWER_FILTER_OLD_WEIGHT) + voltage_mv + (POWER_FILTER_TOTAL / 2U)) / POWER_FILTER_TOTAL;
  }

  return s_filtered_voltage_mv;
}

/**
 * @brief 计算指定电量百分比对应的标称电压。
 *
 * @param[in] percent 电量百分比，范围：0 到 100。
 *
 * @return 百分比对应的标称电压，单位：mV。
 */
static uint32_t Power_PercentCenterMv(uint8_t percent)
{
  /* Power_CalcPercent 的反函数(线性)，供滞回判定使用。 */
  if (percent == 0U) { return POWER_EMPTY_MV; }
  if (percent >= 100U) { return POWER_FULL_MV; }
  return POWER_EMPTY_MV + (((uint32_t)percent * (POWER_FULL_MV - POWER_EMPTY_MV)) / 100U);
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
static uint8_t Power_PercentPassesHysteresis(uint8_t previous, uint8_t candidate, uint32_t filtered_voltage_mv)
{
  uint32_t center_mv = Power_PercentCenterMv(previous);

  if (candidate > previous) { return (filtered_voltage_mv >= (center_mv + POWER_PERCENT_HYSTERESIS_MV)) ? 1U : 0U; }
  return ((filtered_voltage_mv + POWER_PERCENT_HYSTERESIS_MV) <= center_mv) ? 1U : 0U;
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
static uint8_t Power_ApplyPercentConfirm(uint8_t previous, uint8_t candidate, uint32_t filtered_voltage_mv)
{
  if (candidate == previous) {
    s_pending_percent_count = 0U;
    return previous;
  }

  if (Power_PercentPassesHysteresis(previous, candidate, filtered_voltage_mv) == 0U) {
    s_pending_percent_count = 0U;
    return previous;
  }

  if (s_pending_percent_count < POWER_PERCENT_CONFIRM_COUNT) { s_pending_percent_count++; }
  if (s_pending_percent_count >= POWER_PERCENT_CONFIRM_COUNT) {
    s_pending_percent_count = 0U;
    return candidate;
  }

  return previous;
}

Power_Result_t Sensor_Power_Init(void)
{
  memset(&s_snapshot, 0, sizeof(s_snapshot));
  s_filtered_voltage_mv   = 0U;
  s_pending_percent_count = 0U;
  s_filter_valid          = 0U;
  s_present               = 0U;
  s_initialized           = 1U;
  return POWER_RESULT_OK;
}

void Sensor_Power_RequestReinit(void)
{
  s_reinit_request = 1U;
}

Power_Result_t Sensor_Power_Service(uint32_t now_ms)
{
  uint32_t voltage_mv = 0U;
  uint32_t filtered_voltage_mv;
  uint8_t candidate_percent;
  uint8_t now_present;
  uint8_t fresh_insert;

  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    (void)Sensor_Power_Init();
  }
  if (s_initialized == 0U) { return POWER_RESULT_NO_DATA; }

  if (BSP_ADC_ReadVoltageMv(&voltage_mv) != BSP_STATUS_OK) {
    s_snapshot.error_count++;
    return POWER_RESULT_IO_ERROR;
  }

  /* 接入跳变检测：电压由"未接入"(< 接入门限)跳到"接入"视为热插拔。此时以本次
     读数(BSP 已做 8x 过采样)重新播种滤波并直接采用候选电量、跳过 10 次确认，
     使插上瞬间立即给出真实电量；其后恢复常规滤波/确认以抑制带载抖动。 */
  now_present  = (voltage_mv >= POWER_PRESENT_MV) ? 1U : 0U;
  fresh_insert = ((now_present != 0U) && (s_present == 0U)) ? 1U : 0U;
  s_present    = now_present;

  s_snapshot.rx_sequence++;
  if (s_snapshot.rx_sequence == 0U) { s_snapshot.rx_sequence = 1U; }
  s_snapshot.sample_time_ms = now_ms;

  if (fresh_insert != 0U) {
    s_filter_valid          = 0U;
    s_pending_percent_count = 0U;
  }
  filtered_voltage_mv = Power_FilterVoltage(voltage_mv);
  s_snapshot.voltage_v = ((float)filtered_voltage_mv) / 1000.0f;
  candidate_percent   = Power_CalcPercent(filtered_voltage_mv);
  s_snapshot.percent  = ((s_snapshot.rx_sequence == 1U) || (fresh_insert != 0U)) ? candidate_percent : Power_ApplyPercentConfirm(s_snapshot.percent, candidate_percent, filtered_voltage_mv);
  s_snapshot.low_voltage = (voltage_mv < POWER_EMPTY_MV) ? 1U : 0U;

  return POWER_RESULT_OK;
}

Power_Result_t Sensor_Power_CopySnapshot(Power_Snapshot_t *out)
{
  if (out == 0) { return POWER_RESULT_INVALID_PARAM; }

  *out = s_snapshot;
  return (s_snapshot.rx_sequence != 0U) ? POWER_RESULT_OK : POWER_RESULT_NO_DATA;
}

Power_Result_t Sensor_Power_GetStatus(Power_Status_t *out)
{
  if (out == 0) { return POWER_RESULT_INVALID_PARAM; }

  out->rx_sequence    = s_snapshot.rx_sequence;
  out->sample_time_ms = s_snapshot.sample_time_ms;
  out->error_count    = s_snapshot.error_count;
  out->percent        = s_snapshot.percent;
  out->low_voltage    = s_snapshot.low_voltage;
  out->reserved       = 0U;
  return (s_snapshot.rx_sequence != 0U) ? POWER_RESULT_OK : POWER_RESULT_NO_DATA;
}
