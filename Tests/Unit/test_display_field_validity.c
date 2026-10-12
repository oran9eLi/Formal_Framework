/** @file test_display_field_validity.c @brief 实际 Display 快照消费路径有效性回归，屏幕调用用桩记录。 */
#include <assert.h>
#include <stdio.h>
#include "../../Display/Src/display.c"
static unsigned writes[DISPLAY_HMI_VAR_COUNT], invalidations[DISPLAY_HMI_VAR_COUNT];
static App_NavigationSnapshot_t local_nav;
static Px4Lite_Result_t environment_result=PX4LITE_OK;
Display_Result_t Display_LvglSetValue(Display_HmiVariableId_t id,uint32_t value)
{ (void)value;writes[id]++;return DISPLAY_OK; }
Display_Result_t Display_LvglInvalidateValue(Display_HmiVariableId_t id)
{ invalidations[id]++;return DISPLAY_OK; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out,uint32_t now)
{ (void)now;*out=local_nav;return PX4LITE_OK; }
/** @brief 完整 PrepareSnapshot 回归：主电池模块离线，而电池 2 字段独立有效。 */
Px4Lite_Result_t App_GetDisplayEnvironment(App_EnvironmentSnapshot_t *out,uint32_t now)
{ (void)now;memset(out,0,sizeof(*out));out->valid_mask=APP_ENV_VALID_VOLTAGE2;out->voltage2_mv=12000;return environment_result; }
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t id,Px4Lite_ModuleStatus_t *out)
{ (void)id;memset(out,0,sizeof(*out));out->state=PX4LITE_STATE_OFFLINE;return PX4LITE_OK; }
Px4Lite_Result_t App_GetDisplayNavigation(App_NavigationSnapshot_t *out,uint32_t now)
{ return App_CopyNavigation(out,now); }
Px4Lite_Result_t App_GetDisplaySystem(App_SystemSnapshot_t *out,uint32_t now)
{ (void)out;(void)now;return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayAlarm(App_AlarmSnapshot_t *out,uint32_t now)
{ (void)out;(void)now;return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMotor(App_MotorSnapshot_t *out,uint32_t now)
{ (void)out;(void)now;return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayDateTime(App_DateTimeSnapshot_t *out,uint32_t now)
{ (void)out;(void)now;return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetDisplayMessageLog(App_DisplayLogSnapshot_t *out,uint32_t now)
{ (void)out;(void)now;return PX4LITE_NOT_READY; }
void App_GetDisplayLinkStatus(App_DisplayLinkStatus_t *out,uint32_t now)
{ (void)now;memset(out,0,sizeof(*out)); }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{ (void)out;return PX4LITE_NOT_READY; }
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return PX4LITE_REMOTE_MODE_LOCAL; }
uint8_t App_GetSelectedRemoteNode(uint8_t *id) { *id=0;return 0; }
uint8_t App_GetLocalNodeId(void) { return 1; }
uint8_t App_IsMotorPowerInhibited(void) { return 1; }
uint8_t App_IsMotorPowerLowVoltage(void) { return 0; }
uint32_t Px4Lite_PlatformGetMs(void) { return 1000; }
void App_MessageLogPushMotorPowerCutoff(uint32_t now) { (void)now; }
void App_MessageLogInit(uint32_t now) { (void)now; }
void App_MessageLogUpdate(uint32_t now) { (void)now; }
void Display_PagesClearLogMessages(void) {}
void Display_PagesPushLogMessage(Display_LogMsg_t msg,uint32_t time) { (void)msg;(void)time; }
uint32_t Display_PagesGetLogVersion(void) { return 0; }
void Display_LvglSetAlarmRows(const uint32_t *rows,uint16_t count) { (void)rows;(void)count; }
Display_Result_t Display_LvglInit(uint32_t now) { (void)now;return DISPLAY_OK; }
Display_Result_t Display_LvglSetMotorPulseUs(uint8_t index,uint16_t pulse) { (void)index;(void)pulse;return DISPLAY_OK; }
int main(void)
{
  App_NavigationSnapshot_t nav;
  App_EnvironmentSnapshot_t env;
  unsigned old_writes;
  memset(&nav,0,sizeof(nav));memset(&env,0,sizeof(env));
  nav.valid_mask=PX4LITE_NAV_VALID_ATTITUDE;nav.roll_deg100=1200;nav.yaw_deg100=3000;
  Display_LoadNavigationSnapshot(&nav);
  assert(s_hmi_values[DISPLAY_HMI_VAR_ROLL].valid && s_hmi_values[DISPLAY_HMI_VAR_YAW].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && !s_hmi_values[DISPLAY_HMI_VAR_ALTITUDE].valid);
  nav.valid_mask|=PX4LITE_NAV_VALID_POSITION;nav.latitude_e7=123456;
  Display_LoadNavigationSnapshot(&nav);old_writes=writes[DISPLAY_HMI_VAR_LATITUDE];
  assert(s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid);
  nav.valid_mask=0;Display_LoadNavigationSnapshot(&nav);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && invalidations[DISPLAY_HMI_VAR_LATITUDE]);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_YAW].valid);
  nav.valid_mask=PX4LITE_NAV_VALID_POSITION;Display_LoadNavigationSnapshot(&nav);
  assert(s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid && writes[DISPLAY_HMI_VAR_LATITUDE]==old_writes+1);
  Display_ClearNavigationSnapshot();assert(!s_hmi_values[DISPLAY_HMI_VAR_LATITUDE].valid);
  env.valid_mask=APP_ENV_VALID_BATTERY1;env.voltage_mv=12000;env.current_ma=100;env.battery_percent=70;
  s_motor_band=DISPLAY_MOTOR_BAND_OK;Display_UpdateBatteryBands(&env);Display_LoadEnvironmentSnapshot(&env);
  assert(s_motor_band==DISPLAY_MOTOR_BAND_OK && Display_EvalMotorFault(&env)==0);
  assert(s_hmi_values[DISPLAY_HMI_VAR_BATTERY_VOLTAGE].valid && s_hmi_values[DISPLAY_HMI_VAR_BATTERY_PERCENT].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && !s_hmi_values[DISPLAY_HMI_VAR_TEMPERATURE].valid);
  env.valid_mask=APP_ENV_VALID_PRESSURE_TEMPERATURE;env.temperature_c=25;env.pressure_pa=100000;
  Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_TEMPERATURE].valid && s_hmi_values[DISPLAY_HMI_VAR_PRESSURE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_HUMIDITY].valid && !s_hmi_values[DISPLAY_HMI_VAR_BATTERY_VOLTAGE].valid);
  env.valid_mask=APP_ENV_VALID_HUMIDITY;env.relative_humidity_pct=50;
  Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_HUMIDITY].valid && !s_hmi_values[DISPLAY_HMI_VAR_PRESSURE].valid);
  env.valid_mask=APP_ENV_VALID_BATTERY2;env.voltage2_mv=12000;env.current2_ma=200;env.battery2_percent=60;
  Display_UpdateBatteryBands(&env);Display_LoadEnvironmentSnapshot(&env);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && !s_hmi_values[DISPLAY_HMI_VAR_BATTERY_VOLTAGE].valid);
  env.valid_mask=APP_ENV_VALID_CURRENT2|APP_ENV_VALID_PERCENT2;s_motor_band=DISPLAY_MOTOR_BAND_DISCONNECT;
  Display_LoadEnvironmentSnapshot(&env);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid);
  assert(Display_EvalMotorFault(&env)==0 && !s_hmi_values[DISPLAY_HMI_VAR_SELF_CHECK_MOTOR].valid);
  env.valid_mask=APP_ENV_VALID_VOLTAGE1;env.voltage_mv=10000;
  Display_UpdateBatteryBands(&env);assert(Display_EvalMainFault(&env)==PX4LITE_FAULT_POWER_CHARGE);
  env.valid_mask=0;assert(Display_EvalMainFault(&env)==0);
  Display_LoadEnvironmentSnapshot(&env);assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid);
  env.valid_mask=APP_ENV_VALID_CURRENT2;old_writes=writes[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT];
  Display_LoadEnvironmentSnapshot(&env);assert(writes[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT]==old_writes+1);
  Display_ClearEnvironmentFields();Display_ClearBatteryFields();assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_CURRENT].valid);
  local_nav.valid_mask=PX4LITE_NAV_VALID_TIME;local_nav.gnss_utc_date=260928;local_nav.gnss_utc_sec=0;
  Display_LoadFallbackDateTime(1000);assert(s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid);
  local_nav.valid_mask=PX4LITE_NAV_VALID_ATTITUDE;Display_LoadFallbackDateTime(1000);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_GNSS_TIME].valid && !s_hmi_values[DISPLAY_HMI_VAR_CLOCK_TIME].valid && !s_hmi_values[DISPLAY_HMI_VAR_DATE].valid);
  s_display_ready=1;s_display_initialized=1;
  assert(Display_PrepareSnapshot(1000)==DISPLAY_OK);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_BATTERY_VOLTAGE].valid);
  old_writes=writes[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE];
  environment_result=PX4LITE_STALE;
  assert(Display_PrepareSnapshot(1100)==DISPLAY_OK);
  assert(!s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid);
  environment_result=PX4LITE_OK;
  assert(Display_PrepareSnapshot(1200)==DISPLAY_OK);
  assert(s_hmi_values[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE].valid && writes[DISPLAY_HMI_VAR_MOTOR_BAT_VOLTAGE]==old_writes+1);
  puts("display field validity passed");return 0;
}
