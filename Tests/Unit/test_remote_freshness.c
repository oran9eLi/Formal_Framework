/** @file test_remote_freshness.c
 * @brief 验证独立远端数据域不会借其他域刷新旧字段。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Business/Src/app_display_model.c"
static Px4Lite_RemoteTelemetry_t remote_source;
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return PX4LITE_REMOTE_MODE_REMOTE; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyAttitudeStatus(App_AttitudeStatus_t *out)
{ (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyRemoteTelemetry(Px4Lite_RemoteTelemetry_t *out)
{ *out = remote_source; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *out, uint32_t now)
{ (void)out; (void)now; return PX4LITE_NOT_READY; }
int main(void)
{
  App_NavigationSnapshot_t nav;
  App_EnvironmentSnapshot_t env;
  memset(&remote_source, 0, sizeof(remote_source));
  remote_source.header.valid = 1U;
  remote_source.header.sample_time_ms = 10000U;
  remote_source.valid_mask = PX4LITE_REMOTE_VALID_NAVIGATION | PX4LITE_REMOTE_VALID_ATTITUDE;
  remote_source.stale_mask = PX4LITE_REMOTE_VALID_NAVIGATION;
  remote_source.navigation_valid_mask = PX4LITE_NAV_VALID_POSITION | PX4LITE_NAV_VALID_VELOCITY;
  remote_source.position_update_ms = 10000U - PX4LITE_REMOTE_DATA_STALE_MS - 1U;
  remote_source.velocity_update_ms = remote_source.position_update_ms;
  remote_source.attitude_update_ms = 10000U;
  remote_source.latitude_e7 = 123456789;
  remote_source.gnss_fix_type = 3U;
  remote_source.roll_deg100 = 500;
  assert(App_GetDisplayNavigation(&nav, 10000U) == PX4LITE_OK);
  assert((nav.valid_mask & PX4LITE_NAV_VALID_POSITION) == 0U);
  assert(nav.latitude_e7 == 0 && nav.gnss_fix_type == 0U);
  assert(nav.roll_deg100 == 500);
  remote_source.valid_mask = PX4LITE_REMOTE_VALID_ENVIRONMENT | PX4LITE_REMOTE_VALID_BATTERY;
  remote_source.stale_mask = PX4LITE_REMOTE_VALID_ENVIRONMENT;
  remote_source.environment_update_ms = 10000U - PX4LITE_REMOTE_DATA_STALE_MS - 1U;
  remote_source.environment_valid_mask = PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE;
  remote_source.pressure_temperature_update_ms = remote_source.environment_update_ms;
  remote_source.battery_update_ms = 10000U;
  remote_source.battery1_update_ms = 10000U;
  remote_source.battery_valid_mask = PX4LITE_REMOTE_BATTERY_VALID_PRIMARY;
  remote_source.pressure_pa = 101325.0f;
  remote_source.voltage_mv = 12000U;
  assert(App_GetDisplayEnvironment(&env, 10000U) == PX4LITE_OK);
  assert(env.pressure_pa == 0.0f && env.voltage_mv == 12000U);
  assert((env.valid_mask & APP_ENV_VALID_BATTERY2) == 0U);
  remote_source.environment_valid_mask |= PX4LITE_REMOTE_ENV_VALID_HUMIDITY;
  remote_source.humidity_update_ms = 10000U; remote_source.relative_humidity_pct = 45.0f;
  remote_source.battery_valid_mask = PX4LITE_REMOTE_BATTERY_VALID_CURRENT1 | PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2;
  remote_source.battery2_update_ms = remote_source.environment_update_ms;
  remote_source.current_ma = 125U; remote_source.voltage2_mv = 15000U;
  assert(App_GetDisplayEnvironment(&env,10000U) == PX4LITE_OK);
  assert(env.valid_mask == (APP_ENV_VALID_CURRENT1 | APP_ENV_VALID_HUMIDITY));
  assert(env.voltage_mv == 0U && env.voltage2_mv == 0U && env.current_ma == 125);
  assert(env.pressure_pa == 0.0f && env.relative_humidity_pct == 45.0f);
  remote_source.navigation_valid_mask = PX4LITE_NAV_VALID_TIME | PX4LITE_NAV_VALID_ALTITUDE;
  remote_source.time_update_ms = 10000U; remote_source.altitude_update_ms = remote_source.position_update_ms;
  remote_source.attitude_update_ms = remote_source.position_update_ms;
  remote_source.valid_mask |= PX4LITE_REMOTE_VALID_NAVIGATION | PX4LITE_REMOTE_VALID_ATTITUDE;
  remote_source.gnss_utc_date = 260928U; remote_source.gnss_utc_sec = 0U;
  assert(App_GetDisplayNavigation(&nav,10000U) == PX4LITE_OK);
  assert(nav.valid_mask == PX4LITE_NAV_VALID_TIME && nav.gnss_utc_date == 260928U && nav.gnss_utc_sec == 0U);
  assert(nav.roll_deg100 == 0 && nav.altitude_mm == 0 && nav.latitude_e7 == 0);
  puts("remote freshness tests passed"); return 0;
}
