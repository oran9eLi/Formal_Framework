#include <stdint.h>
#include <stdio.h>

#include "bsp_status.h"
#include "sensor_power.h"

static uint32_t s_voltage_mv;

BSP_Status_t BSP_ADC_ReadVoltageMv(uint32_t *voltage_mv)
{
  if (voltage_mv == 0) { return BSP_STATUS_ERROR; }
  *voltage_mv = s_voltage_mv;
  return BSP_STATUS_OK;
}

static uint32_t SnapshotVoltageMv(void)
{
  Power_Snapshot_t snapshot;

  if (Sensor_Power_CopySnapshot(&snapshot) != POWER_RESULT_OK) { return 0U; }
  return (uint32_t)((snapshot.voltage_v * 1000.0f) + 0.5f);
}

static int ExpectUint32(const char *label, uint32_t actual, uint32_t expected)
{
  if (actual != expected) {
    printf("%s: expected %lu, got %lu\n", label, (unsigned long)expected, (unsigned long)actual);
    return 0;
  }
  return 1;
}

int main(void)
{
  int ok = 1;

  s_voltage_mv = 10000U;
  (void)Sensor_Power_Init();
  if (Sensor_Power_Service(1000U) != POWER_RESULT_OK) {
    printf("service failed\n");
    return 1;
  }

  ok &= ExpectUint32("calibrated and compensated voltage", SnapshotVoltageMv(), 10200U);

  if (ok != 0) {
    printf("power adc configurable calibration tests passed\n");
    return 0;
  }

  return 1;
}
