/** @file test_storage_freshness.c
 * @brief 使用真实记录生产与 CSV 格式化，验证断更数据不冒充新测量。
 */
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include "../../Framework/Src/px4lite_storage.c"
#include "../../Storage/Src/storage_csv.c"
#include "../../Storage/Src/storage_file.c"
static Px4Lite_SensorBaro_t test_baro;
static Px4Lite_Result_t baro_result;
static uint8_t other_sources_ready;
static Px4Lite_SensorGnss_t test_gnss;
static Px4Lite_VehicleNavigation_t test_navigation;
static Px4Lite_BatteryStatus_t test_battery;
static Px4Lite_MotorOutputs_t test_motor;
static char saved_line[STORAGE_CSV_LINE_MAX];
/** @brief 读取真实 CSV 字段，测试预期列号独立于生产格式化实现。 */
static long Field(unsigned column)
{
  const char *cursor = saved_line;
  while (column-- != 0U) { cursor = strchr(cursor, ','); assert(cursor != 0); cursor++; }
  return strtol(cursor, 0, 10);
}
/* 其余外设此用例均未接入。真实格式化器输出由真实生产者调用。 */
uint8_t Storage_TimeIsValid(void) { return 0U; }
uint32_t Storage_TimeDateYmd(void) { return 0U; }
uint32_t Storage_TimeTimeHhmmss(void) { return 0U; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { *out = test_navigation; return other_sources_ready ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { *out = test_gnss; return other_sources_ready ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { *out = test_baro; return baro_result; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { *out = test_battery; return other_sources_ready ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { *out = test_battery; return other_sources_ready ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { *out = test_motor; return other_sources_ready ? PX4LITE_OK : PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out) { memset(out, 0, sizeof(*out)); }
uint16_t StorageQueue_Count(void) { return 0U; }
uint32_t StorageQueue_DropCount(void) { return 0U; }
Storage_EventAction_t StorageEventTracker_Update(const char *source, uint32_t fault, const char *message, uint8_t active, uint32_t *count)
{ (void)source; (void)fault; (void)message; (void)active; (void)count; return STORAGE_EVENT_ACTION_NONE; }
Px4Lite_Result_t StorageQueue_Push(const Storage_Record_t *record)
{
  if (record->type == STORAGE_RECORD_DATA) { strcpy(saved_line, record->line); }
  return PX4LITE_OK;
}
int main(void)
{
  memset(&test_baro, 0, sizeof(test_baro));
  test_baro.header.valid = 1U; test_baro.header.sample_time_ms = 1000U;
  test_baro.temperature_c = 25.0f; test_baro.pressure_pa = 100000.0f;
  baro_result = PX4LITE_OK;
  Storage_ProduceDataRecord(1000U);
  assert(strstr(saved_line, "+25.00") != 0);
  assert(Field(36) == 2 && Field(41) == 3 && Field(42) == 1000);
  Storage_ProduceDataRecord(1200U);
  assert(Field(0) == 1200 && Field(41) == 3 && Field(42) == 1000); /* 新行、相同有效样本。 */
  Storage_ProduceDataRecord(10000U);
  assert(strstr(saved_line, "+25.00") == 0); /* 原实现在新时间戳下仍重复旧温度。 */
  assert(Field(41) == 2 && Field(42) == 1000);
  test_baro.header.sample_time_ms = 10000U; test_baro.temperature_c = 0.0f;
  Storage_ProduceDataRecord(10000U);
  assert(Field(41) == 3 && Field(42) == 10000 && Field(10) == 0); /* 有效的零摄氏度。 */
  test_baro.header.valid = 0U; Storage_ProduceDataRecord(10000U);
  assert(Field(41) == 1);
  test_baro.header.valid = 1U; test_baro.temperature_c = NAN; Storage_ProduceDataRecord(10000U);
  assert(Field(41) == 1);
  baro_result = PX4LITE_NOT_READY; Storage_ProduceDataRecord(10000U);
  assert(Field(41) == 0 && Field(42) == 0);
  other_sources_ready = 1U;
  test_gnss.header.valid = 1U; test_gnss.header.sample_time_ms = 1000U; test_gnss.fix_type = 1U;
  test_gnss.latitude_e7 = 12345;
  test_navigation.header.valid = 1U; test_navigation.header.sample_time_ms = 10000U;
  test_navigation.valid_mask = PX4LITE_NAV_VALID_ATTITUDE | PX4LITE_NAV_VALID_POSITION;
  test_navigation.latitude_e7 = 12345; test_navigation.roll_deg100 = 3000;
  test_battery.header.valid = 1U; test_battery.header.sample_time_ms = 1000U; test_battery.voltage_mv = 12000U;
  test_motor.header.valid = 1U; test_motor.header.sample_time_ms = 1000U; test_motor.duty_percent[0] = 50U;
  Storage_ProduceDataRecord(10000U);
  assert(Field(37) == 2 && Field(38) == 1000 && Field(4) == 0 && Field(5) == 0);
  assert(Field(39) == 3 && Field(40) == 10000 && Field(7) == 30);
  assert(Field(43) == 2 && Field(45) == 2 && Field(47) == 2);
  assert(Field(13) == 0 && Field(18) == 0 && Field(23) == 0);
  test_gnss.header.sample_time_ms = test_battery.header.sample_time_ms = test_motor.header.sample_time_ms = 10000U;
  Storage_ProduceDataRecord(10000U);
  assert(Field(37) == 3 && Field(43) == 3 && Field(45) == 3 && Field(47) == 3);
  assert(Field(5) == 12345 && Field(13) == 12 && Field(18) == 12 && Field(23) == 50);
  test_navigation.valid_mask = PX4LITE_NAV_VALID_POSITION; test_gnss.fix_type = 0U;
  test_battery.header.valid = test_motor.header.valid = 0U;
  Storage_ProduceDataRecord(10000U);
  assert(Field(37) == 1 && Field(39) == 1 && Field(43) == 1 && Field(45) == 1 && Field(47) == 1);
  {
    Storage_CsvData_t maximum;
    char path[32];
    unsigned commas = 0U;
    const char *at;
    /* 确保全部 uint32/int32 极值与状态都不会截断一行。 */
    memset(&maximum, 0xff, sizeof(maximum));
    maximum.latitude_e7 = INT32_MIN; maximum.longitude_e7 = INT32_MIN;
    maximum.roll_deg100 = INT32_MIN; maximum.pitch_deg100 = INT32_MIN; maximum.yaw_deg100 = INT32_MIN;
    maximum.temperature_c100 = INT32_MIN; maximum.current_ma = INT32_MIN; maximum.current2_ma = INT32_MIN;
    assert(StorageCsv_FormatDataLine(&maximum, saved_line, sizeof(saved_line)) == PX4LITE_OK);
    for (at = saved_line; *at; ++at) { if (*at == ',') { commas++; } }
    assert(commas == 48U);
    assert(StorageCsv_FormatDataLine(&maximum, saved_line, 16U) == PX4LITE_OVERFLOW);
    assert(StorageFile_FormatPath(STORAGE_RECORD_DATA, 20260915U, path, sizeof(path)) == PX4LITE_OK);
    assert(strcmp(path, "0:/260915_2.CSV") == 0);
    assert(StorageFile_FormatPath(STORAGE_RECORD_DATA, 0U, path, sizeof(path)) == PX4LITE_OK);
    assert(strcmp(path, "0:/UNSYNC_2.CSV") == 0);
    assert(StorageFile_FormatPath(STORAGE_RECORD_EVENT, 20260915U, path, sizeof(path)) == PX4LITE_OK);
    assert(strcmp(path, "0:/260915_E.CSV") == 0);
  }
  puts("storage freshness tests passed");
  return 0;
}
