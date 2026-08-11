#include <stdint.h>
#include <stdio.h>

#include "bsp_config.h"
#include "bsp_status.h"
#include "bsp_adc_current.h"
#include "sensor_power.h"

static const uint32_t *s_voltage_sequence;
static uint32_t s_voltage_count;
static uint32_t s_voltage_index;

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
  if ((voltage_mv == 0) || (s_voltage_sequence == 0) || (s_voltage_count == 0U)) { return BSP_STATUS_ERROR; }

  if (s_voltage_index >= s_voltage_count) {
    *voltage_mv = s_voltage_sequence[s_voltage_count - 1U];
  } else {
    *voltage_mv = s_voltage_sequence[s_voltage_index];
    s_voltage_index++;
  }
  return BSP_STATUS_OK;
}

/* 本用例只验证电压分压和电量曲线，不涉及电流。电流计引脚电压固定桩为 0mV，
   等效读到 0A，不影响任何电压/电量断言。缺此桩会导致 sensor_power.c 链接失败。 */
BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv)
{
  (void)channel;
  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }
  *voltage_mv = 0U;
  return BSP_STATUS_OK;
}

static void LoadVoltageSequence(const uint32_t *sequence, uint32_t count)
{
  s_voltage_sequence = sequence;
  s_voltage_count    = count;
  s_voltage_index    = 0U;
}

static uint32_t SnapshotVoltageMv(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 0U; }
  return (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
}

static uint8_t SnapshotPercent(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 255U; }
  return snapshot.percent;
}

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int TestDividerMatchesMeasuredCalibration(void)
{
  int ok = 1;

  /* 2026-07-24 地面站标定值 10.17793941，按 1e-4 精度锁定为 101779/10000。 */
  ok &= ExpectUint32("divider numerator", BSP_ADC_DIVIDER_NUM, 101779U);
  ok &= ExpectUint32("divider denominator", BSP_ADC_DIVIDER_DEN, 10000U);
  return ok;
}

static int TestPublishedVoltageIsRawCalibratedMillivolts(void)
{
  /* 契约：snapshot.voltage_v 发布的是每帧原始标定电压（默认标定 1:1、0 偏移），
     不做滤波；一阶低通只作用于电量百分比与低压判断（见 sensor_power.c:365）。
     因此在默认标定下，发布电压逐帧等于输入电压。 */
  static const uint32_t voltages_mv[] = {10750U, 10790U, 10740U, 10760U};
  static const uint32_t expected_mv[] = {10750U, 10790U, 10740U, 10760U};
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(voltages_mv, (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])));
  (void)Sensor_Power_Init();

  for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])); ++i) {
    if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("raw calibrated millivolt voltage output", SnapshotVoltageMv(), expected_mv[i]);
  }
  return ok;
}

static int TestBatteryPercentClampsAndUsesFivePercentSteps(void)
{
  /* 期望值按 sensor_power_config.h 的真实非线性电量表(0%=9900mV,5%=10200,10%=10500,
     20%=10800,...,90%=12450,100%=12550mV)分段线性插值后 5% 取整得到：
       9800/9900 ≤ 0% 点 → 0；10199/10200 落 0~5% 段 → 5；10499/10500 落 5~10% 段 → 10；
       11525 落 40~50% 段，插值 44 → 取整 45；12600 > 100% 点 → 100。 */
  static const uint32_t voltages_mv[] = {9800U, 9900U, 10199U, 10200U, 10499U, 10500U, 11525U, 12600U};
  static const uint8_t expected_pct[] = {0U, 0U, 5U, 5U, 10U, 10U, 45U, 100U};
  uint32_t i;
  int ok = 1;

  for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])); ++i) {
    LoadVoltageSequence(&voltages_mv[i], 1U);
    (void)Sensor_Power_Init();
    if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("service failed for percent clamp sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("five-percent battery step", SnapshotPercent(), expected_pct[i]);
  }

  return ok;
}

static int TestBatteryLowVoltageFlagNeedsSustainedConfirm(void)
{
  /* 低压标志按 sensor_power_config.h：进入门限 POWER_LOW_WARNING_MV(10300mV)、
     解除门限 POWER_LOW_RECOVER_MV(10500mV)、需连续 POWER_LOW_CONFIRM_COUNT(=10) 次确认。
     单次跨门限不会立即翻转，故本用例持续喂同一电压足够多帧后只校验最终稳定态。
     两阶段之间不重新 Init，以真实检验"从已置位状态恢复"。 */
  static const uint32_t low_mv  = 10000U; /* < 10300，持续低于进入门限 */
  static const uint32_t high_mv = 12000U; /* > 10500，持续高于解除门限 */
  Power_Snapshot_t snapshot;
  uint32_t t = 0U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&low_mv, 1U);
  (void)Sensor_Power_Init();
  for (i = 0U; i < 20U; ++i) {
    if (Sensor_Power_Service((++t) * 1000U) != POWER_RESULT_OK) { printf("low-phase service failed\n"); return 0; }
  }
  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { printf("low-phase snapshot failed\n"); return 0; }
  ok &= ExpectUint32("low voltage flag sets after sustained low", snapshot.low_voltage, 1U);

  LoadVoltageSequence(&high_mv, 1U);
  for (i = 0U; i < 40U; ++i) {
    if (Sensor_Power_Service((++t) * 1000U) != POWER_RESULT_OK) { printf("recover-phase service failed\n"); return 0; }
  }
  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { printf("recover-phase snapshot failed\n"); return 0; }
  ok &= ExpectUint32("low voltage flag clears after sustained recover", snapshot.low_voltage, 0U);

  return ok;
}

static int TestBatteryPercentRequiresTenConsecutiveNewSteps(void)
{
  /* 起始 11525mV 在真实曲线上为 45%（40~50% 段插值 44 取整 45）；升到 12550mV 后，
     需连续 POWER_PERCENT_CONFIRM_COUNT(=10) 次确认才更新。注意确认翻转时的候选档位是按
     "当拍滤波电压"重算的：第 10 次确认(第 11 拍)时滤波值约 12492mV(未完全收敛到 12550)，
     落 90~100% 段插值 94 取整 95，故确认后档位为 95% 而非 100%。 */
  static const uint32_t initial_mv = 11525U;
  static const uint32_t higher_mv  = 12550U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&initial_mv, 1U);
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("initial service failed\n");
    return 0;
  }
  ok &= ExpectUint32("initial battery step", SnapshotPercent(), 45U);

  LoadVoltageSequence(&higher_mv, 1U);
  for (i = 0U; i < 9U; ++i) {
    if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK) {
      printf("pre-confirm service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery step before confirm", SnapshotPercent(), 45U);
  }

  if (Sensor_Power_Service(11000U) != POWER_RESULT_OK) {
    printf("confirm service failed\n");
    return 0;
  }
  ok &= ExpectUint32("battery step after ten confirms", SnapshotPercent(), 95U);

  return ok;
}

static int TestBatteryPercentIgnoresBoundaryNoise(void)
{
  /* 起始 11525mV → 45%；噪声 11640mV 的候选档位为 50%，但其滤波电压始终达不到
     45% 中心电压(11550mV)+滞回(200mV)=11750mV 的翻转门限，故档位应稳定保持 45%。 */
  static const uint32_t initial_mv = 11525U;
  static const uint32_t noisy_mv   = 11640U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&initial_mv, 1U);
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("initial service failed for boundary noise test\n");
    return 0;
  }
  ok &= ExpectUint32("initial boundary step", SnapshotPercent(), 45U);

  LoadVoltageSequence(&noisy_mv, 1U);
  for (i = 0U; i < 12U; ++i) {
    if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK) {
      printf("noise service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery step should ignore boundary noise", SnapshotPercent(), 45U);
  }

  return ok;
}

static int TestBatteryReinsertPublishesImmediately(void)
{
  static const uint32_t absent_mv   = 0U;     /* 拔出/未接入：电压远低于接入门限 */
  static const uint32_t inserted_mv = 12550U; /* 热插入满电电池 */
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&absent_mv, 1U);
  (void)Sensor_Power_Init();
  for (i = 0U; i < 3U; ++i) {
    if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("absent service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
  }
  ok &= ExpectUint32("absent battery percent", SnapshotPercent(), 0U);

  /* 热插拔接入：框架在 health 检测到模块恢复时调用 Px4Lite_BatteryRecover ->
     Sensor_Power_RequestReinit（见 px4lite_modules.c / platform_f407.c），下一次 Service
     经 Init 走 rx_sequence==1 快路径，直接发布真实电量，而不是从 0 经 10 次确认缓慢爬升。
     本用例按该真实路径先请求重初始化，再喂入接入电压。 */
  Sensor_Power_RequestReinit();
  LoadVoltageSequence(&inserted_mv, 1U);
  if (Sensor_Power_Service(4000U) != POWER_RESULT_OK) {
    printf("reinsert service failed\n");
    return 0;
  }
  ok &= ExpectUint32("reinserted battery percent jumps immediately", SnapshotPercent(), 100U);

  return ok;
}

int main(void)
{
  int ok = 1;

  ok &= TestDividerMatchesMeasuredCalibration();
  ok &= TestPublishedVoltageIsRawCalibratedMillivolts();
  ok &= TestBatteryPercentClampsAndUsesFivePercentSteps();
  ok &= TestBatteryLowVoltageFlagNeedsSustainedConfirm();
  ok &= TestBatteryPercentRequiresTenConsecutiveNewSteps();
  ok &= TestBatteryPercentIgnoresBoundaryNoise();
  ok &= TestBatteryReinsertPublishesImmediately();

  if (ok != 0) {
    printf("power adc calibration tests passed\n");
    return 0;
  }

  return 1;
}
