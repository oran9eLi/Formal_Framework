/** @file test_remote_navigation.c
 * @brief 真实 MAVLink 解码覆盖独立导航字段、非法值、UTC 午夜及回绕。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_mavlink_rx.c"
static Px4Lite_RemoteTelemetry_t remote;
static mavlink_message_t message;
static void Named(const char *name, int32_t value, uint32_t seconds, uint32_t now)
{
  mavlink_named_value_int_t packet;
  memset(&packet, 0, sizeof(packet)); strncpy(packet.name, name, sizeof(packet.name));
  packet.value = value; packet.time_boot_ms = seconds;
  remote.last_rx_ms = now;
  mavlink_msg_named_value_int_encode(42U, 1U, &message, &packet);
  assert(MavRx_DecodeNamedValueInt(&remote, &message, now) != 0U);
}
int main(void)
{
  mavlink_gps_raw_int_t gps;
  mavlink_sys_status_t sys;
  mavlink_battery_status_t battery;
  mavlink_scaled_pressure_t pressure;
  uint32_t later = PX4LITE_REMOTE_DATA_STALE_MS + 1U;
  memset(&gps, 0, sizeof(gps)); gps.fix_type = 3U;
  gps.lat = 225000000; gps.lon = 1135000000; gps.alt = 25000;
  gps.vel = 100U; gps.cog = 9000U;
  mavlink_msg_gps_raw_int_encode(42U, 1U, &message, &gps);
  assert(MavRx_DecodeGpsRaw(&remote, &message) != 0U);
  assert(remote.position_update_ms == 0U);
  assert((remote.navigation_valid_mask & PX4LITE_NAV_VALID_POSITION) != 0U);
  Named("GNSSUTC", 260928, 0U, later);
  Named("BAROALT", 30000, 0U, later);
  Named("GNSS_SAT", 8, 0U, later);
  Px4Lite_RemoteTelemetryRefreshFreshness(&remote, later);
  assert((remote.navigation_stale_mask & PX4LITE_NAV_VALID_POSITION) != 0U);
  assert((remote.navigation_stale_mask & PX4LITE_NAV_VALID_VELOCITY) != 0U);
  assert((remote.navigation_stale_mask & (PX4LITE_NAV_VALID_TIME | PX4LITE_NAV_VALID_ALTITUDE)) == 0U);
  assert(remote.gnss_utc_sec == 0U && remote.gnss_utc_date == 260928U);
  Named("GNSSUTC", 260231, 1U, later);
  assert((remote.navigation_valid_mask & PX4LITE_NAV_VALID_TIME) == 0U);
  gps.fix_type = 1U;
  mavlink_msg_gps_raw_int_encode(42U, 1U, &message, &gps);
  assert(MavRx_DecodeGpsRaw(&remote, &message) != 0U);
  assert((remote.navigation_valid_mask & PX4LITE_NAV_VALID_POSITION) == 0U);
  remote.navigation_valid_mask = PX4LITE_NAV_VALID_POSITION;
  remote.position_update_ms = UINT32_MAX - 100U;
  Px4Lite_RemoteTelemetryRefreshFreshness(&remote, 50U);
  assert(remote.navigation_stale_mask == 0U);
  Px4Lite_RemoteTelemetryRefreshFreshness(&remote, PX4LITE_REMOTE_DATA_STALE_MS);
  assert((remote.navigation_stale_mask & PX4LITE_NAV_VALID_POSITION) != 0U);
  memset(&remote, 0, sizeof(remote));
  memset(&sys, 0, sizeof(sys)); sys.voltage_battery=12000U; sys.current_battery=-1; sys.battery_remaining=-1;
  mavlink_msg_sys_status_encode(42U,1U,&message,&sys);
  assert(MavRx_DecodeSysStatus(&remote,&message)!=0U);
  assert(remote.battery_valid_mask==PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE1);
  memset(&battery,0,sizeof(battery)); battery.id=1U; battery.voltages[0]=15000U; battery.current_battery=-1; battery.battery_remaining=80;
  remote.last_rx_ms=1U;
  mavlink_msg_battery_status_encode(42U,1U,&message,&battery);
  assert(MavRx_DecodeBattery(&remote,&message)!=0U);
  assert((remote.battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_VOLTAGE2)!=0U);
  assert((remote.battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_CURRENT2)==0U);
  remote.last_rx_ms=later+1U;
  mavlink_msg_sys_status_encode(42U,1U,&message,&sys);
  assert(MavRx_DecodeSysStatus(&remote,&message)!=0U);
  Px4Lite_RemoteTelemetryRefreshFreshness(&remote,later+1U);
  assert((remote.battery_stale_mask&PX4LITE_REMOTE_BATTERY_VALID_SECONDARY)!=0U);
  assert((remote.battery_stale_mask&PX4LITE_REMOTE_BATTERY_VALID_PRIMARY)==0U);
  battery.voltages[0]=UINT16_MAX; battery.current_battery=-1; battery.battery_remaining=-1;
  mavlink_msg_battery_status_encode(42U,1U,&message,&battery);
  assert(MavRx_DecodeBattery(&remote,&message)!=0U);
  assert((remote.battery_valid_mask&PX4LITE_REMOTE_BATTERY_VALID_SECONDARY)==0U);
  assert(remote.voltage2_mv==0U && remote.current2_ma==0U && remote.battery2_percent==0U);
  sys.voltage_battery=UINT16_MAX; sys.current_battery=10; sys.battery_remaining=-1;
  mavlink_msg_sys_status_encode(42U,1U,&message,&sys);
  assert(MavRx_DecodeSysStatus(&remote,&message)!=0U);
  assert(remote.battery_valid_mask==PX4LITE_REMOTE_BATTERY_VALID_CURRENT1 && remote.voltage_mv==0U);
  memset(&remote,0,sizeof(remote));
  memset(&pressure,0,sizeof(pressure)); pressure.press_abs=1013.25f; pressure.temperature=2500;
  mavlink_msg_scaled_pressure_encode(42U,1U,&message,&pressure);
  assert(MavRx_DecodePressure(&remote,&message)!=0U);
  Named("HUMIDITY",500,0U,later);
  Px4Lite_RemoteTelemetryRefreshFreshness(&remote,later);
  assert((remote.environment_stale_mask&PX4LITE_REMOTE_ENV_VALID_PRESSURE_TEMPERATURE)!=0U);
  assert((remote.environment_stale_mask&PX4LITE_REMOTE_ENV_VALID_HUMIDITY)==0U);
  memset(&remote,0,sizeof(remote)); Named("HUMIDITY",500,0U,0U);
  assert(remote.environment_valid_mask==PX4LITE_REMOTE_ENV_VALID_HUMIDITY);
  Named("HUMIDITY",-1,0U,1U); assert(remote.environment_valid_mask==0U);
  puts("remote navigation tests passed"); return 0;
}
