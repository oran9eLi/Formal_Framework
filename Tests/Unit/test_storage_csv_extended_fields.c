/**
 * @file test_storage_csv_extended_fields.c
 * @brief 验证周期 DATA CSV 扩展字段的表头和格式化结果。
 */

#include <stdio.h>
#include <string.h>

#include "storage_csv.h"

static int ExpectContains(const char *name, const char *text, const char *needle)
{
  if (strstr(text, needle) == 0) {
    printf("FAIL %s missing=%s\n", name, needle);
    return 1;
  }
  return 0;
}

static int TestExtendedHeader(void)
{
  const char *header = StorageCsv_DataHeader();
  int failures       = 0;

  failures += ExpectContains("header local date", header, "local_date");
  failures += ExpectContains("header motor", header, "motor1_pct");
  failures += ExpectContains("header alarm", header, "active_alarm_count");
  failures += ExpectContains("header lora", header, "lora_parse_error_count");
  failures += ExpectContains("header storage", header, "storage_drop_count");
  return failures;
}

static int TestExtendedDataLine(void)
{
  Storage_CsvData_t data;
  char line[384];
  int failures = 0;

  memset(&data, 0, sizeof(data));
  data.time_ms                = 1234U;
  data.local_date_ymd         = 20260622U;
  data.local_time_hhmmss      = 153045U;
  data.time_sync_state        = 2U;
  data.gnss_valid             = 1U;
  data.latitude_e7            = 321234567L;
  data.longitude_e7           = 1181234567L;
  data.roll_deg100            = 123;
  data.pitch_deg100           = -456;
  data.temperature_c100       = 2789;
  data.pressure_hpa100        = 101325U;
  data.humidity_pct100        = 5566U;
  data.voltage_mv             = 11890U;
  data.battery_pct            = 87U;
  data.motor_pct[0]           = 10U;
  data.motor_pct[1]           = 20U;
  data.motor_pct[2]           = 30U;
  data.motor_pct[3]           = 40U;
  data.motor_run_state        = 1U;
  data.active_alarm_count     = 2U;
  data.highest_fault_code     = 0x2301U;
  data.lora_rx_count          = 11U;
  data.lora_tx_count          = 12U;
  data.lora_parse_error_count = 3U;
  data.lora_send_error_count  = 4U;
  data.storage_queue_count    = 5U;
  data.storage_drop_count     = 6U;

  if (StorageCsv_FormatDataLine(&data, line, sizeof(line)) != PX4LITE_OK) {
    printf("FAIL format result\n");
    return 1;
  }

  failures += ExpectContains("line date time sync", line, "20260622,153045,2");
  failures += ExpectContains("line motors", line, ",10,20,30,40,1,");
  failures += ExpectContains("line alarm", line, ",2,8961,");
  failures += ExpectContains("line lora storage", line, ",11,12,3,4,5,6");
  return failures;
}

int main(void)
{
  int failures = 0;

  failures += TestExtendedHeader();
  failures += TestExtendedDataLine();

  if (failures != 0) {
    printf("storage csv extended field tests failed: %d\n", failures);
    return 1;
  }

  printf("storage csv extended field tests passed\n");
  return 0;
}
