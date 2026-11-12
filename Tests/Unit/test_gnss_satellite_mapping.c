/**
 * @file test_gnss_satellite_mapping.c
 * @brief 使用真实Adapter与App验证逐星小页映射、只读所有权和独立时效。
 * @details 借用已有HAL类型替身；不读取串口、不执行传感器服务，不把副本零值当无数据。
 */
#define main ExistingDiagnosticsMappingMain
#include "test_gnss_diagnostics_mapping.c"
#undef main
#include "../../Business/Src/app_data_api.c"

static Gnss_SatellitePage_t s_sat_source;
static Gnss_Result_t s_sat_result = GNSS_RESULT_OK;
static unsigned s_sat_copies;
static uint32_t s_sat_clock;

/** @brief Driver替身只交付缓存；记录页号，证明App没有执行硬件采集。 */
Gnss_Result_t Sensor_GNSS_CopySatellitePage(uint8_t page, Gnss_SatellitePage_t *out)
{ ++s_sat_copies; *out = s_sat_source; assert(page == 0U); return s_sat_result; }
/** @brief 仅用于复制被sensor抢占时复核真实时间，不改变源received_ms。 */
uint32_t BSP_Time_GetTickMs(void) { return s_sat_clock; }

/** @brief 合法0ms/0角度/0载噪比及跨层字段逐项保留，不把它们混为未知。 */
int main(void)
{
  App_GnssSatellitePage_t out;
  assert(App_CopyGnssSatellitePage(0U, NULL, 0U) == PX4LITE_INVALID_PARAM);
  assert(s_sat_copies == 0U);
#if PX4LITE_ENABLE_GNSS
  memset(&s_sat_source, 0, sizeof(s_sat_source));
  s_sat_source.available = 1U; s_sat_source.total = 65U; s_sat_source.stored = 64U;
  s_sat_source.count = 2U; s_sat_source.truncated = 1U; s_sat_source.used_count = 20U;
  s_sat_source.sequence = UINT32_MAX; s_sat_source.itow_ms = 6000U;
  s_sat_source.entries[0].gnss_id = 0U; s_sat_source.entries[0].sv_id = 12U;
  s_sat_source.entries[0].used = 1U;
  s_sat_source.entries[0].valid_mask = GNSS_SAT_VALID_ELEVATION | GNSS_SAT_VALID_AZIMUTH | 0x80U;
  s_sat_source.entries[1].gnss_id = 3U; s_sat_source.entries[1].sv_id = 12U;
  s_sat_source.entries[1].elevation_deg = -5; s_sat_source.entries[1].azimuth_deg = 360;
  s_sat_source.entries[1].cno_dbhz = 42U; s_sat_source.entries[1].quality = 4U;
  s_sat_source.entries[1].valid_mask = GNSS_SAT_VALID_ELEVATION | GNSS_SAT_VALID_AZIMUTH;
  assert(App_CopyGnssSatellitePage(0U, &out, 0U) == PX4LITE_OK);
  assert(out.available && out.received_ms == 0U && out.sequence == UINT32_MAX);
  assert(out.total == 65U && out.stored == 64U && out.count == 2U && out.truncated && out.used_count == 20U);
  assert(out.entries[0].cno_dbhz == 0U && out.entries[0].quality == 0U && out.entries[0].elevation_deg == 0);
  assert(out.entries[0].valid_mask == 3U && out.entries[0].used == 1U);
  assert(out.entries[0].gnss_id != out.entries[1].gnss_id && out.entries[0].sv_id == out.entries[1].sv_id);
  assert(out.entries[1].azimuth_deg == 360 && out.entries[1].elevation_deg == -5 && out.entries[1].quality == 4U);
  assert(out.entries[2].sv_id == 0U && out.entries[2].valid_mask == 0U);
  assert(s_sat_copies == 1U && s_driver_copies == 0U && s_transport_copies == 0U);
  assert(App_CopyGnssSatellitePage(0U, &out, APP_GNSS_SAT_MAX_AGE_MS) == PX4LITE_OK);
  s_sat_clock = APP_GNSS_SAT_MAX_AGE_MS + 1U;
  assert(App_CopyGnssSatellitePage(0U, &out, s_sat_clock) == PX4LITE_STALE && !out.available);
  s_sat_source.received_ms = 1001U; s_sat_clock = 1001U;
  assert(App_CopyGnssSatellitePage(0U, &out, 1000U) == PX4LITE_OK && out.received_ms == 1001U);
  s_sat_source.received_ms = UINT32_MAX - 9U;
  assert(App_CopyGnssSatellitePage(0U, &out, 10U) == PX4LITE_OK);
  s_sat_source.count = 6U;
  assert(App_CopyGnssSatellitePage(0U, &out, 10U) == PX4LITE_IO_ERROR && !out.available);
  s_sat_source.count = 2U; s_sat_result = GNSS_RESULT_IO_ERROR;
  assert(App_CopyGnssSatellitePage(0U, &out, 10U) == PX4LITE_IO_ERROR && !out.available);
  s_sat_result = GNSS_RESULT_INVALID_PARAM;
  assert(App_CopyGnssSatellitePage(0U, &out, 10U) == PX4LITE_INVALID_PARAM && !out.available);
  s_sat_result = GNSS_RESULT_NO_DATA;
  assert(App_CopyGnssSatellitePage(0U, &out, 10U) == PX4LITE_NOT_READY && !out.available);
#else
  assert(App_CopyGnssSatellitePage(0U, &out, 0U) == PX4LITE_NOT_READY);
  assert(s_sat_copies == 0U && s_driver_copies == 0U && s_transport_copies == 0U);
#endif
  puts("GNSS satellite mapping: PASS (five-item copy, actual zeros, independent age, disabled ownership)");
  return 0;
}
