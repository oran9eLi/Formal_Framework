#include <stdint.h>
#include <stdio.h>

#include "bsp_config.h"
#include "bsp_status.h"
#include "bsp_adc_current.h"
#include "sensor_power_config.h"
#include "sensor_power.h"

static const uint32_t *s_voltage_sequence;
static uint32_t s_voltage_count;
static uint32_t s_voltage_index;
static const uint32_t *s_current_sequence;
static uint32_t s_current_count;
static uint32_t s_current_index;

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

BSP_Status_t BSP_ADC_Current_ReadVoltageMv(BSP_ADC_CurrentChannel_t channel, uint32_t *voltage_mv)
{
  (void)channel;
  if ((voltage_mv == 0) || (s_current_sequence == 0) || (s_current_count == 0U)) { return BSP_STATUS_ERROR; }

  if (s_current_index >= s_current_count) {
    *voltage_mv = s_current_sequence[s_current_count - 1U];
  } else {
    *voltage_mv = s_current_sequence[s_current_index];
    s_current_index++;
  }
  return BSP_STATUS_OK;
}

static void LoadVoltageSequence(const uint32_t *sequence, uint32_t count)
{
  s_voltage_sequence = sequence;
  s_voltage_count    = count;
  s_voltage_index    = 0U;
}

static void LoadCurrentSequence(const uint32_t *sequence, uint32_t count)
{
  s_current_sequence = sequence;
  s_current_count    = count;
  s_current_index    = 0U;
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

static uint8_t SnapshotLowVoltage(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 255U; }
  return snapshot.low_voltage;
}

static int32_t SnapshotCurrentMa(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return -1; }
  return snapshot.current_ma;
}

static uint32_t SnapshotPowerMw(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 0U; }
  return snapshot.power_mw;
}

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int ExpectInt32(const char *label, int32_t actual, int32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %ld, got %ld\n", label, (long)expected, (long)actual);
    return 0;
  }
  return 1;
}

static int TestDividerMatchesMeasuredCalibration(void)
{
  int ok = 1;

  ok &= ExpectUint32("divider numerator", BSP_ADC_DIVIDER_NUM, 10080U);
  ok &= ExpectUint32("divider denominator", BSP_ADC_DIVIDER_DEN, 1000U);
  return ok;
}

static int TestPublishedVoltageKeepsMillivoltPrecision(void)
{
  static const uint32_t voltages_mv[] = {10750U, 10790U, 10740U, 10760U};
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(voltages_mv, (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])));
  (void)Sensor_Power_Init();

  for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])); ++i) {
    if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("millivolt voltage output", SnapshotVoltageMv(), voltages_mv[i]);
  }
  return ok;
}

static int TestBatteryCurrentAndPowerUseCurrentAdcMv(void)
{
  static const uint32_t voltage_mv = 12000U;
  static const uint32_t current_mv = POWER_CURRENT_ZERO_MV + POWER_CURRENT_MV_PER_A;
  int ok = 1;

  LoadVoltageSequence(&voltage_mv, 1U);
  LoadCurrentSequence(&current_mv, 1U);
  (void)Sensor_Power_Init();

  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("service failed for current conversion test\n");
    return 0;
  }

  ok &= ExpectInt32("current output", SnapshotCurrentMa(), 1000);
  ok &= ExpectUint32("power output", SnapshotPowerMw(), 12000U);
  return ok;
}

static int TestBatteryPercentClampsAndUsesFivePercentSteps(void)
{
  static const uint32_t voltages_mv[] = {9800U, 9900U, 10200U, 10500U, 10800U, 11100U, 11400U, 11700U, 11950U, 12150U, 12300U, 12450U, 12550U, 12600U};
  static const uint8_t expected_pct[] = {0U, 0U, 5U, 10U, 20U, 30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U, 100U};
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

static int TestBatteryLowVoltageRequiresConsecutiveWarningSamples(void)
{
  static const uint32_t safe_mv = POWER_LOW_RECOVER_MV + 100U;
  static const uint32_t low_mv  = POWER_LOW_WARNING_MV - 2500U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&safe_mv, 1U);
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("initial service failed for low voltage confirm test\n");
    return 0;
  }
  ok &= ExpectUint32("initial low voltage flag", SnapshotLowVoltage(), 0U);

  LoadVoltageSequence(&low_mv, 1U);
  for (i = 0U; i < (POWER_LOW_CONFIRM_COUNT - 1U); ++i) {
    if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK) {
      printf("pre-confirm low voltage service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("low voltage flag before confirm", SnapshotLowVoltage(), 0U);
  }

  if (Sensor_Power_Service(12000U) != POWER_RESULT_OK) {
    printf("confirm low voltage service failed\n");
    return 0;
  }
  ok &= ExpectUint32("low voltage flag after confirm", SnapshotLowVoltage(), 1U);

  return ok;
}

static int TestBatteryLowVoltageUsesRecoverHysteresis(void)
{
  static const uint32_t low_mv      = POWER_LOW_WARNING_MV - 2500U;
  static const uint32_t boundary_mv = POWER_LOW_WARNING_MV + 100U;
  static const uint32_t recover_mv  = POWER_LOW_RECOVER_MV + 2000U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&low_mv, 1U);
  (void)Sensor_Power_Init();
  for (i = 0U; i < POWER_LOW_CONFIRM_COUNT; ++i) {
    if (Sensor_Power_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("low voltage setup failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
  }
  ok &= ExpectUint32("low voltage setup flag", SnapshotLowVoltage(), 1U);

  LoadVoltageSequence(&boundary_mv, 1U);
  for (i = 0U; i < (POWER_LOW_CONFIRM_COUNT + 2U); ++i) {
    if (Sensor_Power_Service((i + 20U) * 1000U) != POWER_RESULT_OK) {
      printf("boundary recovery service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("low voltage flag keeps hysteresis", SnapshotLowVoltage(), 1U);
  }

  LoadVoltageSequence(&recover_mv, 1U);
  for (i = 0U; i < (POWER_LOW_CONFIRM_COUNT - 1U); ++i) {
    if (Sensor_Power_Service((i + 40U) * 1000U) != POWER_RESULT_OK) {
      printf("pre-confirm recovery service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("low voltage flag before recover confirm", SnapshotLowVoltage(), 1U);
  }

  if (Sensor_Power_Service(60000U) != POWER_RESULT_OK) {
    printf("recover confirm service failed\n");
    return 0;
  }
  ok &= ExpectUint32("low voltage flag after recover confirm", SnapshotLowVoltage(), 0U);

  return ok;
}

static int TestBatteryPercentRequiresTenConsecutiveNewSteps(void)
{
  static const uint32_t initial_mv = 11700U;
  static const uint32_t higher_mv  = 12550U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&initial_mv, 1U);
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("initial service failed\n");
    return 0;
  }
  ok &= ExpectUint32("initial battery step", SnapshotPercent(), 50U);

  LoadVoltageSequence(&higher_mv, 1U);
  for (i = 0U; i < 9U; ++i) {
    if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK) {
      printf("pre-confirm service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery step before confirm", SnapshotPercent(), 50U);
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
  static const uint32_t initial_mv = 11700U;
  static const uint32_t noisy_mv   = 11820U;
  uint32_t i;
  int ok = 1;

  LoadVoltageSequence(&initial_mv, 1U);
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("initial service failed for boundary noise test\n");
    return 0;
  }
  ok &= ExpectUint32("initial boundary step", SnapshotPercent(), 50U);

  LoadVoltageSequence(&noisy_mv, 1U);
  for (i = 0U; i < 12U; ++i) {
    if (Sensor_Power_Service((i + 2U) * 1000U) != POWER_RESULT_OK) {
      printf("noise service failed at sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery step should ignore boundary noise", SnapshotPercent(), 50U);
  }

  return ok;
}

int main(void)
{
  int ok = 1;

  ok &= TestDividerMatchesMeasuredCalibration();
  ok &= TestPublishedVoltageKeepsMillivoltPrecision();
  ok &= TestBatteryCurrentAndPowerUseCurrentAdcMv();
  ok &= TestBatteryPercentClampsAndUsesFivePercentSteps();
  ok &= TestBatteryLowVoltageRequiresConsecutiveWarningSamples();
  ok &= TestBatteryLowVoltageUsesRecoverHysteresis();
  ok &= TestBatteryPercentRequiresTenConsecutiveNewSteps();
  ok &= TestBatteryPercentIgnoresBoundaryNoise();

  if (ok != 0) {
    printf("power adc calibration tests passed\n");
    return 0;
  }

  return 1;
}
