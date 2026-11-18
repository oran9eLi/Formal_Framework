/**
 * @file test_single_supply_csv.c
 * @brief 真实生产者与格式化器验证禁用一路不采集/不报故障，主回路列和未知SOC保持正确。
 */
#define main StorageLegacyTestMain
#include "test_storage_freshness.c"
#undef main

/** @brief 返回指定 CSV 列的首字节，保留空列语义。 */
static const char *Column(unsigned column)
{
  const char *at = saved_line;
  while (column-- != 0U) { at = strchr(at, ','); assert(at != 0); ++at; }
  return at;
}

int main(void)
{
  const char *at;
  unsigned commas = 0U;
  assert(!PX4LITE_ENABLE_BATTERY1 && PX4LITE_ENABLE_BATTERY2);
  other_sources_ready = 1U;
  test_battery.header.valid = 1U;
  test_battery.header.sample_time_ms = 1000U;
  test_battery.voltage_mv = 12000U;
  test_battery.current_ma = 1500;
  test_battery.current_valid = 1U;
  test_battery.percent = 77U; /* 通用供电不可宣称已识别电池并据此上报77%。 */
  Storage_ProduceDataRecord(1000U);
  assert(battery1_copies == 0U && battery1_events == 0U && battery2_copies == 1U);
  assert(Field(43) == STORAGE_SAMPLE_MISSING && Field(44) == 0);
  assert(Field(45) == STORAGE_SAMPLE_VALID && Field(46) == 1000);
  assert(Field(13) == 0 && Field(14) == 0 && Field(15) == 0);
  assert(strncmp(Column(18), "12.000,", 7U) == 0);
  assert(strncmp(Column(19), "+1.500,", 7U) == 0);
  assert(strncmp(Column(20), "18.000,", 7U) == 0);
  assert(*Column(21) == ','); /* 既不显示255%，也不伪造0%。 */
  for (at = saved_line; *at != '\0'; ++at) { if (*at == ',') { ++commas; } }
  assert(commas == 48U);
  Storage_ProduceDataRecord(10000U);
  assert(Field(45) == STORAGE_SAMPLE_STALE && Field(18) == 0);
  assert(battery1_copies == 0U && battery1_events == 0U);
  test_battery.header.sample_time_ms = 10000U;
  Storage_ProduceDataRecord(10000U);
  assert(Field(45) == STORAGE_SAMPLE_VALID && *Column(21) == ',');
  assert(battery1_events == 0U);
  puts("single_supply_csv: PASS (49 columns, actual input2, unknown SOC blank, no disabled-source fault)");
  return 0;
}
