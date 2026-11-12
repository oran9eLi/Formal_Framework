/**
 * @file test_display_gnss_status_utc.c
 * @brief 真实Display门面验证独立GNSS状态、UTC及冻结；不依赖Navigation位置有效。
 * @details App替身提供明确授权的各域有效位，真实App裁期由clean GNSS测试另外执行。
 *          不访问硬件、不读取诊断、不创建任务或改变电机命令。
 */
#include <assert.h>
#include <stdio.h>
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display.c"

static App_GnssDetails_t s_gnss;
static Px4Lite_Result_t s_gnss_result = PX4LITE_OK;
static App_NavigationSnapshot_t s_navigation;
static App_DateTimeSnapshot_t s_clock;
static Px4Lite_Result_t s_clock_result = PX4LITE_OK;
static unsigned s_gnss_copies;
static uint8_t s_frozen;

Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value)
{ (void)id; (void)value; return DISPLAY_OK; }
Display_Result_t Display_LvglInvalidateValue(Display_HmiVariableId_t id)
{ (void)id; return DISPLAY_OK; }
Px4Lite_Result_t App_CopyGnssDetails(App_GnssDetails_t *out, uint32_t now_ms)
{ (void)now_ms; *out = s_gnss; s_gnss_copies++; return s_gnss_result; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; *out = s_navigation; return PX4LITE_OK; }
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now_ms)
{ return App_CopyNavigation(out, now_ms); }
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; *out = s_clock; return s_clock_result; }
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now_ms)
{ (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{ (void)id; memset(out, 0, sizeof(*out)); out->state = PX4LITE_STATE_OFFLINE; return PX4LITE_OK; }
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now_ms)
{
  (void)now_ms; memset(out, 0, sizeof(*out));
  out->valid_mask = APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2;
  out->voltage2_mv = 12000U; out->current2_ma = 1000;
  return PX4LITE_OK;
}
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms)
{ (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now_ms)
{ (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
void App_GetDisplayLinkStatus(App_DisplayLinkStatus_t *out, uint32_t now_ms)
{ (void)now_ms; memset(out, 0, sizeof(*out)); }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{ (void)out; return PX4LITE_NOT_READY; }
uint8_t App_GetLocalNodeId(void) { return 1U; }
Px4Lite_Result_t App_GetMotorLocked(uint8_t *locked) { *locked = 1U; return PX4LITE_OK; }
uint32_t Px4Lite_PlatformGetMs(void) { return 0U; }
uint32_t Display_PagesGetLogVersion(void) { return 0U; }
void Display_PagesSetLogSnapshot(const App_DisplayLogSnapshot_t *snapshot) { (void)snapshot; }
void Display_LvglSetLogSnapshotVersion(uint32_t version) { (void)version; }
void Display_LvglSetAlarmSnapshot(const App_AlarmSnapshot_t *alarm, const uint32_t *rows, uint16_t count)
{ (void)alarm; (void)rows; (void)count; }
uint8_t Display_LvglObservationFrozen(void) { return s_frozen; }
Display_Result_t Display_LvglInit(uint32_t now_ms) { (void)now_ms; return DISPLAY_OK; }
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t index, uint16_t pulse)
{ (void)index; (void)pulse; return DISPLAY_OK; }
Display_Result_t Display_LvglInvalidateMotorPulseUs(uint8_t index)
{ (void)index; return DISPLAY_OK; }

/** @brief 验证No fix与真实0/2/12颗均可见，只有状态域有效才授权，Navigation不能代填。 */
static void TestIndependentState(void)
{
  static const uint8_t counts[] = {0U, 2U, 12U};
  unsigned i;
  memset(&s_gnss, 0, sizeof(s_gnss)); memset(&s_navigation, 0, sizeof(s_navigation));
  s_gnss_result = PX4LITE_OK; s_gnss.valid_mask = PX4LITE_GNSS_VALID_NAV_STATUS;
  for (i = 0U; i < sizeof(counts); ++i) {
    s_gnss.satellites_used = counts[i]; Display_LoadLocalGnssSnapshot(0U);
    assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].valid && s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 0U);
    assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].valid && s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].value == counts[i]);
  }
  s_gnss.fix_type = 1U; s_gnss.fix_dimension = 2U; Display_LoadLocalGnssSnapshot(0U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 2U);
  s_gnss.fix_dimension = 3U; Display_LoadLocalGnssSnapshot(0U); assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 3U);
  s_gnss.fix_type = 2U; Display_LoadLocalGnssSnapshot(0U); assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 4U);
  s_navigation.valid_mask = PX4LITE_NAV_VALID_POSITION; s_navigation.gnss_fix_type = 99U; s_navigation.satellites_used = 99U;
  s_navigation.latitude_e7 = 123456789; s_navigation.hdop_x100 = 0U;
  Display_LoadNavigationSnapshot(&s_navigation);
  assert(s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && !s_hmi_values[DISPLAY_HMI_VAR_GNSS_HDOP].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 4U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].value == 12U);
  /* App已撤掉过期状态位；新ACK的last_rx不能授权旧sat。 */
  s_gnss.valid_mask = 0U; s_gnss.last_rx_ms = 9000U; Display_LoadLocalGnssSnapshot(9000U);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].valid && !s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].valid);
  s_gnss.valid_mask = PX4LITE_GNSS_VALID_NAV_STATUS; s_gnss_result = PX4LITE_STALE; Display_LoadLocalGnssSnapshot(9000U);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].valid && !s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].valid);
}

/** @brief GPS UTC直接显示午夜和12:34:56；统一本地钟与原UTC+8兜底不能写GPS UTC字段。 */
static void TestUtcAndLocalClock(void)
{
  s_gnss_result = PX4LITE_OK; s_gnss.valid_mask = PX4LITE_GNSS_VALID_TIME;
  s_gnss.utc_sec = 0U; Display_LoadLocalGnssSnapshot(0U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid && s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 0U);
  s_clock.local_date_ymd = 20261106U; s_clock.local_time_hhmmss = 80000U;
  Display_LoadDateTimeSnapshot(&s_clock);
  assert(s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 80000U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 0U);
  s_gnss.utc_sec = 45296U; Display_LoadLocalGnssSnapshot(1000U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 123456U);
  s_clock.local_time_hhmmss = 203456U; Display_LoadDateTimeSnapshot(&s_clock);
  assert(s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 203456U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 123456U);
  s_navigation.valid_mask = PX4LITE_NAV_VALID_TIME; s_navigation.gnss_utc_sec = 45296U; s_navigation.gnss_utc_date = 261106U;
  Display_LoadFallbackDateTime(1000U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 203456U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 123456U);
  s_navigation.valid_mask = 0U; Display_LoadFallbackDateTime(1000U);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].valid && !s_hmi_values[DISPLAY_HMI_VAR_DATE].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid && s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 123456U);
  s_gnss.utc_sec = 86400U; Display_LoadLocalGnssSnapshot(1000U); assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid);
}

/** @brief Prepare只读一次GNSS；冻结保留GNSS测量值，顶栏Clock/Date和保护反馈仍更新。 */
static void TestPrepareFreeze(void)
{
  unsigned before;
  memset(&s_navigation, 0, sizeof(s_navigation)); memset(&s_gnss, 0, sizeof(s_gnss));
  s_display_ready = s_display_initialized = 1U; s_frozen = 0U; s_gnss_result = PX4LITE_OK;
  s_gnss.valid_mask = PX4LITE_GNSS_VALID_NAV_STATUS | PX4LITE_GNSS_VALID_TIME; s_gnss.satellites_used = 2U;
  s_clock_result = PX4LITE_OK; s_clock.local_date_ymd = 20261106U; s_clock.local_time_hhmmss = 80000U;
  before = s_gnss_copies; assert(Display_PrepareSnapshot(0U) == DISPLAY_OK && s_gnss_copies == before + 1U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 0U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].value == 2U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 0U);
  s_frozen = 1U; s_gnss.fix_type = 1U; s_gnss.fix_dimension = 3U; s_gnss.satellites_used = 12U; s_gnss.utc_sec = 45296U;
  s_clock.local_time_hhmmss = 203456U; before = s_gnss_copies;
  assert(Display_PrepareSnapshot(1000U) == DISPLAY_OK && s_gnss_copies == before);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 0U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].value == 2U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 0U && s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 203456U);
  s_frozen = 0U; assert(Display_PrepareSnapshot(1001U) == DISPLAY_OK && s_gnss_copies == before + 1U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].value == 3U && s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].value == 12U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].value == 123456U && s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 203456U);
  s_gnss_result = PX4LITE_NOT_READY; assert(Display_PrepareSnapshot(2000U) == DISPLAY_OK);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_FIX].valid && !s_hmi_values[DISPLAY_HMI_VAR_GNSS_SAT_COUNT].valid && !s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].valid && s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].value == 203456U);
}

int main(void)
{
  TestIndependentState(); TestUtcAndLocalClock(); TestPrepareFreeze();
  puts("GNSS normal status/UTC: PASS (no-fix 0/2/12, state bit, UTC/local clock, freeze, single App read)");
  return 0;
}
