/**
 * @file test_display_field_validity.c
 * @brief 通过真实 Display 快照消费路径验证单路输入、逐字段有效性及失效恢复。
 * @details 本文件仅供 host 使用；App 和 LCD 由确定性替身隔离，不执行硬件 I/O。
 */
#include <assert.h>
#include <stdio.h>
/* 仅补齐当前资源快照头使用的host任务类型，不启动调度器。 */
typedef int BaseType_t;
typedef void *TaskHandle_t;
#include "../../Display/Src/display.c"

static unsigned writes[DISPLAY_HMI_VAR_COUNT], invalidations[DISPLAY_HMI_VAR_COUNT];
static App_NavigationSnapshot_t local_nav;
static App_EnvironmentSnapshot_t supply;
static Px4Lite_Result_t environment_result = PX4LITE_OK;

Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id, uint32_t value)
{ (void)value; writes[id]++; return DISPLAY_OK; }
Display_Result_t Display_LvglInvalidateValue(Display_HmiVariableId_t id)
{ invalidations[id]++; return DISPLAY_OK; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now)
{ (void)now; *out = local_nav; return PX4LITE_OK; }
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now)
{ (void)now; *out = supply; return environment_result; }
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t id, Px4Lite_ModuleStatus_t *out)
{ (void)id; memset(out, 0, sizeof(*out)); out->state = PX4LITE_STATE_OFFLINE; return PX4LITE_OK; }
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out, uint32_t now)
{ return App_CopyNavigation(out, now); }
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out, uint32_t now)
{ (void)now; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
void App_GetDisplayLinkStatus(App_DisplayLinkStatus_t *out, uint32_t now)
{ (void)now; memset(out, 0, sizeof(*out)); }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{ (void)out; return PX4LITE_NOT_READY; }
/** @brief 本用例无独立GNSS状态/UTC源，不能让旧Navigation或系统钟伪造其有效位。 */
Px4Lite_Result_t App_CopyGnssDetails(App_GnssDetails_t *out, uint32_t now)
{ (void)now; memset(out, 0, sizeof(*out)); return PX4LITE_NOT_READY; }
uint8_t App_GetLocalNodeId(void) { return 1U; }
Px4Lite_Result_t App_GetMotorLocked(uint8_t *locked) { *locked = 1U; return PX4LITE_OK; }
uint32_t Px4Lite_PlatformGetMs(void) { return 1000U; }
uint32_t Display_PagesGetLogVersion(void) { return 0U; }
void Display_PagesSetLogSnapshot(const App_DisplayLogSnapshot_t *snapshot) { (void)snapshot; }
void Display_LvglSetLogSnapshotVersion(uint32_t version) { (void)version; }
void Display_LvglSetAlarmSnapshot(const App_AlarmSnapshot_t *alarm, const uint32_t *rows, uint16_t count)
{ (void)alarm; (void)rows; (void)count; }
uint8_t Display_LvglObservationFrozen(void) { return 0U; }
Display_Result_t Display_LvglInit(uint32_t now) { (void)now; return DISPLAY_OK; }
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t index, uint16_t pulse)
{ (void)index; (void)pulse; return DISPLAY_OK; }
Display_Result_t Display_LvglInvalidateMotorPulseUs(uint8_t index)
{ (void)index; return DISPLAY_OK; }

/** @brief 不同导航子域的失效与同值恢复不能互相续期或保留旧值。 */
static void TestNavigationValidity(void)
{
  App_NavigationSnapshot_t nav;
  unsigned old_writes;
  memset(&nav, 0, sizeof(nav));
  nav.valid_mask = PX4LITE_NAV_VALID_ATTITUDE; nav.roll_deg100 = 1200; nav.yaw_deg100 = 3000;
  Display_LoadNavigationSnapshot(&nav);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ROLL].valid && s_hmi_values[DISPLAY_HMI_VAR_YAW].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && !s_hmi_values[DISPLAY_HMI_VAR_ALTITUDE].valid);
  nav.valid_mask |= PX4LITE_NAV_VALID_POSITION; nav.latitude_e7 = 123456;
  Display_LoadNavigationSnapshot(&nav); old_writes = writes[DISPLAY_HMI_VAR_LATITUDE];
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_HDOP].valid); /* NAV-PVT位置有效不代表已收到NAV-DOP。 */
  nav.hdop_x100 = 95U; Display_LoadNavigationSnapshot(&nav);
  assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_HDOP].valid && s_hmi_values[DISPLAY_HMI_VAR_GNSS_HDOP].value == 95U);
  nav.hdop_x100 = 0U; Display_LoadNavigationSnapshot(&nav);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_HDOP].valid);
  nav.valid_mask = 0U; Display_LoadNavigationSnapshot(&nav);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && invalidations[DISPLAY_HMI_VAR_LATITUDE]);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_YAW].valid);
  nav.valid_mask = PX4LITE_NAV_VALID_POSITION; Display_LoadNavigationSnapshot(&nav);
  assert(s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && writes[DISPLAY_HMI_VAR_LATITUDE] == old_writes + 1U);
  Display_ClearNavigationSnapshot(); assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid);
}

/** @brief 当前第一组和SOC必须无效；零占位不能伪装成启用输入。 */
static void AssertDisabledFields(void)
{
  assert(!s_hmi_values[DISPLAY_HMI_VAR_BATTERY_VOLTAGE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_BATTERY_CURRENT].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_BATTERY_PERCENT].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_PERCENT].valid);
}

/** @brief 验证总输入真实V/A/W、合法零电流、测量缺失、低压和高压的显示事实。 */
static void TestSingleSupply(void)
{
  App_EnvironmentSnapshot_t env;
  memset(&env, 0, sizeof(env));
  /* 注入禁用组的旧值，门面仍不得将其授权为有效测量。 */
  env.valid_mask = APP_ENV_VALID_BATTERY1; env.voltage_mv = 12000U; env.current_ma = 900;
  Display_UpdateBatteryBands(&env); Display_LoadEnvironmentSnapshot(&env);
  AssertDisabledFields();
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  assert(Display_EvalMainFault(&env) == 0U);
  env.valid_mask = APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2;
  env.voltage2_mv = 12000U; env.current2_ma = 3000;
  Display_UpdateBatteryBands(&env); Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].value == 36000U);
  AssertDisabledFields();
  env.current2_ma = 0; Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid && s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].value == 0U);
  env.valid_mask = APP_ENV_VALID_VOLTAGE2; Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid && !s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  env.valid_mask = APP_ENV_VALID_CURRENT2; env.current2_ma = 1500; Display_LoadEnvironmentSnapshot(&env);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid && !s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  assert(Display_EvalMotorFault(&env) == 0U && !s_hmi_values[DISPLAY_HMI_VAR_SELF_CHECK_MOTOR].valid);
  env.valid_mask = APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2; env.voltage2_mv = 4500U;
  Display_UpdateBatteryBands(&env); Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].value == 450U); /* 真实4.5V，非在位推断零。 */
  assert(Display_EvalMotorFault(&env) != 0U);
  env.voltage2_mv = 15000U; Display_UpdateBatteryBands(&env); Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].value == 1500U);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].value == 22500U);
  /* 功率必须采用原始样本，而非屏幕量化后的12.0V/1.2A。 */
  env.voltage2_mv = 12049U; env.current2_ma = 1234; Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].value == 14868U);
  AssertDisabledFields();
  Display_ClearBatteryFields();
  assert(!s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
}

/** @brief 环境字段、午夜时间及完整快照路径在失效后必须撤值，并允许同值恢复。 */
static void TestEnvironmentAndSnapshot(void)
{
  App_EnvironmentSnapshot_t env;
  unsigned old_writes;
  memset(&env, 0, sizeof(env));
  env.valid_mask = APP_ENV_VALID_PRESSURE_TEMPERATURE; env.temperature_c = 25.0f; env.pressure_pa = 100000.0f;
  Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_TEMPERATURE].valid && s_hmi_values[DISPLAY_HMI_VAR_PRESSURE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_HUMIDITY].valid);
  env.valid_mask = APP_ENV_VALID_HUMIDITY; env.relative_humidity_pct = 50.0f; Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_HUMIDITY].valid && !s_hmi_values[DISPLAY_HMI_VAR_PRESSURE].valid);
  local_nav.valid_mask = PX4LITE_NAV_VALID_TIME; local_nav.gnss_utc_date = 260928U; local_nav.gnss_utc_sec = 0U;
  Display_LoadFallbackDateTime(1000U); assert(s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid); /* 本地钟兜底不能冒充GPS UTC。 */
  local_nav.valid_mask = PX4LITE_NAV_VALID_ATTITUDE; Display_LoadFallbackDateTime(1000U);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid && !s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].valid && !s_hmi_values[DISPLAY_HMI_VAR_DATE].valid);
  supply.valid_mask = APP_ENV_VALID_VOLTAGE2 | APP_ENV_VALID_CURRENT2;
  supply.voltage2_mv = 12000U; supply.current2_ma = 3000;
  s_display_ready = s_display_initialized = 1U;
  assert(Display_PrepareSnapshot(1000U) == DISPLAY_OK);
  assert(s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  AssertDisabledFields(); old_writes = writes[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE];
  environment_result = PX4LITE_STALE;
  assert(Display_PrepareSnapshot(1100U) == DISPLAY_OK);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && !s_hmi_values[DISPLAY_HMI_VAR_INPUT_POWER_MW].valid);
  environment_result = PX4LITE_OK;
  assert(Display_PrepareSnapshot(1200U) == DISPLAY_OK);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && writes[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE] == old_writes + 1U);
  AssertDisabledFields();
}

/** @brief 当前配置的消费行为回归；不启动调度器或访问屏幕。 */
int main(void)
{
  assert(!PX4LITE_ENABLE_BATTERY1 && PX4LITE_ENABLE_BATTERY2);
  TestNavigationValidity(); TestSingleSupply(); TestEnvironmentAndSnapshot();
  puts("display field validity: PASS (input 2 V/A/W, disabled input 1, unknown/zero, actual voltage, no SOC, recovery)");
  return 0;
}
