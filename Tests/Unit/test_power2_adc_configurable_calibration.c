#include <stdint.h>
#include <stdio.h>

#include "bsp_status.h"
#include "sensor_power_config.h"
#include "sensor_power2.h"

static const uint32_t *s_voltage_sequence;
static uint32_t s_voltage_count;
static uint32_t s_voltage_index;

BSP_Status_t BSP_ADC2_ReadVoltageMv(uint32_t *voltage_mv)
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

static void LoadVoltageSequence(const uint32_t *sequence, uint32_t count)
{
  s_voltage_sequence = sequence;
  s_voltage_count    = count;
  s_voltage_index    = 0U;
}

static uint8_t SnapshotPercent(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power2_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 255U; }
  return snapshot.percent;
}

static uint32_t SnapshotVoltageMv(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power2_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 0U; }
  return (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
}

static uint8_t SnapshotLowVoltage(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power2_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 255U; }
  return snapshot.low_voltage;
}

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

static int TestBattery2UsesMotorBatteryCurve(void)
{
  static const uint32_t voltages_mv[] = {
    POWER2_PERCENT_TABLE_EMPTY_MV,
    POWER2_PERCENT_TABLE_5_MV,
    POWER2_PERCENT_TABLE_10_MV,
    POWER2_PERCENT_TABLE_20_MV,
    POWER2_PERCENT_TABLE_30_MV,
    POWER2_PERCENT_TABLE_40_MV,
    POWER2_PERCENT_TABLE_50_MV,
    POWER2_PERCENT_TABLE_60_MV,
    POWER2_PERCENT_TABLE_70_MV,
    POWER2_PERCENT_TABLE_80_MV,
    POWER2_PERCENT_TABLE_90_MV,
    POWER2_PERCENT_TABLE_FULL_MV
  };
  static const uint8_t expected_pct[] = {0U, 5U, 10U, 20U, 30U, 40U, 50U, 60U, 70U, 80U, 90U, 100U};
  uint32_t i;
  int ok = 1;

  for (i = 0U; i < (uint32_t)(sizeof(voltages_mv) / sizeof(voltages_mv[0])); ++i) {
    LoadVoltageSequence(&voltages_mv[i], 1U);
    (void)Sensor_Power2_Init();
    if (Sensor_Power2_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("battery2 service failed for curve sample %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery2 motor curve percent", SnapshotPercent(), expected_pct[i]);
    ok &= ExpectUint32("battery2 voltage keeps millivolts", SnapshotVoltageMv(), voltages_mv[i]);
  }

  ok &= ExpectUint32("battery2 zero percent voltage", POWER2_PERCENT_TABLE_EMPTY_MV, 10500U);
  return ok;
}

static int TestBattery2LowVoltageFlagDebouncesMotorWarning(void)
{
  uint32_t i;
  int ok = 1;

  static const uint32_t warning_mv = POWER2_LOW_WARNING_MV - 50U;
  LoadVoltageSequence(&warning_mv, 1U);
  (void)Sensor_Power2_Init();

  for (i = 0U; i < (POWER2_LOW_CONFIRM_COUNT - 1U); ++i) {
    if (Sensor_Power2_Service((i + 1U) * 1000U) != POWER_RESULT_OK) {
      printf("battery2 service failed before low voltage confirm %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery2 low voltage before confirm", SnapshotLowVoltage(), 0U);
  }

  if (Sensor_Power2_Service(10000U) != POWER_RESULT_OK) {
    printf("battery2 service failed at low voltage confirm\n");
    return 0;
  }
  ok &= ExpectUint32("battery2 low voltage after confirm", SnapshotLowVoltage(), 1U);

  {
    static const uint32_t recover_mv = POWER2_LOW_RECOVER_MV + 500U;
    LoadVoltageSequence(&recover_mv, 1U);
  }

  for (i = 0U; i < (POWER2_LOW_CONFIRM_COUNT - 1U); ++i) {
    if (Sensor_Power2_Service((i + 11U) * 1000U) != POWER_RESULT_OK) {
      printf("battery2 service failed before low voltage clear %lu\n", (unsigned long)i);
      return 0;
    }
    ok &= ExpectUint32("battery2 low voltage before clear confirm", SnapshotLowVoltage(), 1U);
  }

  if (Sensor_Power2_Service(20000U) != POWER_RESULT_OK) {
    printf("battery2 service failed at low voltage clear\n");
    return 0;
  }
  ok &= ExpectUint32("battery2 low voltage after clear confirm", SnapshotLowVoltage(), 0U);
  ok &= ExpectUint32("battery2 low warning threshold", POWER2_LOW_WARNING_MV, 10800U);
  ok &= ExpectUint32("battery2 low recover threshold", POWER2_LOW_RECOVER_MV, 11000U);
  return ok;
}

int main(void)
{
  int ok = 1;

  ok &= TestBattery2UsesMotorBatteryCurve();
  ok &= TestBattery2LowVoltageFlagDebouncesMotorWarning();

  if (ok != 0) {
    printf("power2 adc configurable calibration tests passed\n");
    return 0;
  }

  return 1;
}
