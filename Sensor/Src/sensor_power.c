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
#include "sensor_power_config.h"

#include <string.h>

/**
 * @brief 电池电量曲线上的一个标定点。
 *
 * @details
 * Sensor_Power 使用有序曲线点把已校准电压换算为电量百分比。表内电压必须从低到高排列，
 * 百分比必须从低到高排列，避免插值和滞回中心点计算出现反向区间。
 */
typedef struct {
  uint32_t voltage_mv; /**< 曲线点对应的电池电压，单位：mV。 */
  uint8_t percent;     /**< 曲线点对应的电量百分比，范围：0 到 100。 */
} Power_PercentPoint_t;

static const Power_PercentPoint_t s_percent_curve[] = {
  {POWER_PERCENT_TABLE_EMPTY_MV, 0U},
  {POWER_PERCENT_TABLE_5_MV, 5U},
  {POWER_PERCENT_TABLE_10_MV, 10U},
  {POWER_PERCENT_TABLE_20_MV, 20U},
  {POWER_PERCENT_TABLE_30_MV, 30U},
  {POWER_PERCENT_TABLE_40_MV, 40U},
  {POWER_PERCENT_TABLE_50_MV, 50U},
  {POWER_PERCENT_TABLE_60_MV, 60U},
  {POWER_PERCENT_TABLE_70_MV, 70U},
  {POWER_PERCENT_TABLE_80_MV, 80U},
  {POWER_PERCENT_TABLE_90_MV, 90U},
  {POWER_PERCENT_TABLE_FULL_MV, 100U}
};

static Power_Snapshot_t s_snapshot;
static uint32_t s_filtered_voltage_mv;
static uint8_t s_pending_percent_count;
static uint8_t s_initialized;
static uint8_t s_filter_valid;
static volatile uint8_t s_reinit_request;

/**
 * @brief 对 BSP 输出的电池电压做二次校准。
 *
 * @details
 * BSP_ADC 负责原始 ADC 到电池电压的基础分压换算。本函数只处理现场标定参数：
 * 比例增益、固定偏移以及可选的负载补偿。默认配置为 1:1、0mV 偏移、关闭负载补偿，
 * 因此不改变既有电压读数。若现场确认开发板工作状态稳定低约 0.2V，可在配置中开启
 * `POWER_LOAD_COMPENSATION_ENABLE` 并设置 `POWER_LOAD_COMPENSATION_MV`。
 *
 * @param[in] measured_mv BSP 层换算出的电池电压，单位：mV。
 *
 * @return 校准后的电池电压，单位：mV；负偏移不会导致返回负值，最低钳位到 0mV。
 */
static uint32_t Power_ApplyCalibration(uint32_t measured_mv)
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
 *
 * @param[in] percent 插值计算得到的原始百分比，范围通常为 0 到 100。
 *
 * @return 按 `POWER_PERCENT_STEP` 四舍五入后的百分比，最大钳位到 100。
 */
static uint8_t Power_RoundToStep(uint32_t percent)
{
  percent = ((percent + (POWER_PERCENT_STEP / 2U)) / POWER_PERCENT_STEP) * POWER_PERCENT_STEP;
  return (percent > 100U) ? 100U : (uint8_t)percent;
}

/**
 * @brief 根据电压毫伏值计算 5% 步进的电量百分比。
 *
 * @param[in] voltage_mv 已滤波电压，单位：mV。
 *
 * @return 电量百分比，范围：0 到 100。
 */
static uint8_t Power_CalcPercent(uint32_t voltage_mv)
{
  uint32_t i;
  uint32_t percent;

  if (voltage_mv <= s_percent_curve[0].voltage_mv) { return s_percent_curve[0].percent; }

  for (i = 1U; i < (uint32_t)(sizeof(s_percent_curve) / sizeof(s_percent_curve[0])); i++) {
    const Power_PercentPoint_t *low = &s_percent_curve[i - 1U];
    const Power_PercentPoint_t *high = &s_percent_curve[i];
    uint32_t range_mv;
    uint32_t range_pct;

    if (voltage_mv > high->voltage_mv) { continue; }

    range_mv = high->voltage_mv - low->voltage_mv;
    range_pct = (uint32_t)high->percent - (uint32_t)low->percent;
    if (range_mv == 0U) { return high->percent; }

    percent = (uint32_t)low->percent + ((((voltage_mv - low->voltage_mv) * range_pct) + (range_mv / 2U)) / range_mv);
    return Power_RoundToStep(percent);
  }

  return s_percent_curve[(sizeof(s_percent_curve) / sizeof(s_percent_curve[0])) - 1U].percent;
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
  uint32_t i;

  if (percent <= s_percent_curve[0].percent) { return s_percent_curve[0].voltage_mv; }

  for (i = 1U; i < (uint32_t)(sizeof(s_percent_curve) / sizeof(s_percent_curve[0])); i++) {
    const Power_PercentPoint_t *low = &s_percent_curve[i - 1U];
    const Power_PercentPoint_t *high = &s_percent_curve[i];
    uint32_t range_mv;
    uint32_t range_pct;

    if (percent > high->percent) { continue; }

    range_mv = high->voltage_mv - low->voltage_mv;
    range_pct = (uint32_t)high->percent - (uint32_t)low->percent;
    if (range_pct == 0U) { return high->voltage_mv; }

    return low->voltage_mv + ((((uint32_t)(percent - low->percent) * range_mv) + (range_pct / 2U)) / range_pct);
  }

  return s_percent_curve[(sizeof(s_percent_curve) / sizeof(s_percent_curve[0])) - 1U].voltage_mv;
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
  s_initialized           = 1U;
  return POWER_RESULT_OK;
}

void Sensor_Power_RequestReinit(void)
{
  s_reinit_request = 1U;
}

Power_Result_t Sensor_Power_Service(uint32_t now_ms)
{
  uint32_t measured_voltage_mv = 0U;
  uint32_t voltage_mv;
  uint32_t filtered_voltage_mv;
  uint8_t candidate_percent;

  if (s_reinit_request != 0U) {
    s_reinit_request = 0U;
    (void)Sensor_Power_Init();
  }
  if (s_initialized == 0U) { return POWER_RESULT_NO_DATA; }

  if (BSP_ADC_ReadVoltageMv(&measured_voltage_mv) != BSP_STATUS_OK) {
    s_snapshot.error_count++;
    return POWER_RESULT_IO_ERROR;
  }

  voltage_mv = Power_ApplyCalibration(measured_voltage_mv);
  s_snapshot.rx_sequence++;
  if (s_snapshot.rx_sequence == 0U) { s_snapshot.rx_sequence = 1U; }
  s_snapshot.sample_time_ms = now_ms;
  s_snapshot.voltage_v      = ((float)voltage_mv) / 1000.0f;
  filtered_voltage_mv       = Power_FilterVoltage(voltage_mv);
  candidate_percent         = Power_CalcPercent(filtered_voltage_mv);
  s_snapshot.percent        = (s_snapshot.rx_sequence == 1U) ? candidate_percent : Power_ApplyPercentConfirm(s_snapshot.percent, candidate_percent, filtered_voltage_mv);
  s_snapshot.low_voltage    = (voltage_mv < POWER_PERCENT_TABLE_EMPTY_MV) ? 1U : 0U;

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
