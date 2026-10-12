/** @file test_m2_remoteid_fields.c @brief RemoteID 速度与协议字段回归。 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_remoteid_tx.c"
static Px4Lite_VehicleNavigation_t fixture_nav;
static Px4Lite_TimeSnapshot_t fixture_time;
static unsigned time_ready, sends;
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { *out=fixture_nav;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { *out=fixture_time;return time_ready?PX4LITE_OK:PX4LITE_NOT_READY; }
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 1; }
Px4Lite_Result_t Px4Lite_MavlinkTxMirrorToRpi(struct __mavlink_message *msg) { (void)msg;return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RemoteIdSend(const uint8_t *data,uint16_t len) { (void)data;assert(len>0);sends++;return PX4LITE_OK; }
int main(void) {
  Px4Lite_VehicleNavigation_t nav;
  mavlink_open_drone_id_location_t loc;
  mavlink_open_drone_id_system_t system;
  uint32_t stamp, before;
  memset(&nav,0,sizeof(nav));nav.valid_mask=PX4LITE_NAV_VALID_VELOCITY;
  nav.velocity_north_cms=100;nav.velocity_east_cms=100;
  assert(RemoteId_HorizontalSpeedCms(&nav)==141);
  assert(RemoteId_DirectionDeg100(&nav)==4500);
  nav.yaw_deg100=27000;assert(RemoteId_DirectionDeg100(&nav)==4500);
  nav.velocity_north_cms=-100;assert(RemoteId_DirectionDeg100(&nav)==13500);
  nav.velocity_east_cms=-100;assert(RemoteId_DirectionDeg100(&nav)==22500);
  nav.velocity_north_cms=100;assert(RemoteId_DirectionDeg100(&nav)==31500);
  nav.velocity_north_cms=0;nav.velocity_east_cms=0;
  assert(RemoteId_HorizontalSpeedCms(&nav)==0 && RemoteId_DirectionDeg100(&nav)==36100);
  nav.velocity_north_cms=INT32_MIN;nav.velocity_east_cms=INT32_MAX;
  assert(RemoteId_HorizontalSpeedCms(&nav)==25425);
  nav.valid_mask=0;assert(RemoteId_HorizontalSpeedCms(&nav)==25500 && RemoteId_DirectionDeg100(&nav)==36100);
  nav.valid_mask=PX4LITE_NAV_VALID_ALTITUDE;assert(RemoteId_SpeedVerticalCms(&nav)==6300);
  assert(RemoteId_Epoch2019(20190101,0,&stamp) && stamp==0);
  assert(RemoteId_Epoch2019(20190102,0,&stamp) && stamp==86400);
  assert(RemoteId_Epoch2019(20200301,0,&stamp) && stamp==425*86400U);
  assert(RemoteId_Epoch2019(20200229,235959,&stamp) && stamp==425*86400U-1);
  assert(!RemoteId_Epoch2019(20190229,0,&stamp));
  assert(!RemoteId_Epoch2019(20181231,0,&stamp));
  assert(!RemoteId_Epoch2019(20200101,240000,&stamp));
  assert(!RemoteId_Epoch2019(21551231,235959,&stamp));
  fixture_nav.header.valid=1;fixture_nav.header.publish_time_ms=1000;fixture_nav.header.sequence=1;
  fixture_nav.valid_mask=PX4LITE_NAV_VALID_POSITION|PX4LITE_NAV_VALID_TIME|PX4LITE_NAV_VALID_VELOCITY;
  fixture_nav.gnss_utc_sec=13*3600+59;fixture_nav.velocity_north_cms=100;fixture_nav.velocity_east_cms=100;
  assert(RemoteId_SendLocation(1000)==PX4LITE_OK);
  mavlink_msg_open_drone_id_location_decode(&s_remoteid_message,&loc);
  assert(loc.timestamp==59.0f && loc.direction==4500 && loc.speed_horizontal==141);
  assert(loc.speed_vertical==6300 && loc.altitude_geodetic==-1000.0f && loc.height==-1000.0f);
  fixture_nav.valid_mask=PX4LITE_NAV_VALID_POSITION;fixture_nav.header.sequence++;
  assert(RemoteId_SendLocation(1000)==PX4LITE_OK);
  mavlink_msg_open_drone_id_location_decode(&s_remoteid_message,&loc);
  assert(loc.timestamp==65535.0f && loc.speed_horizontal==25500 && loc.direction==36100);
  fixture_nav.valid_mask|=PX4LITE_NAV_VALID_TIME;fixture_nav.gnss_utc_sec=0;fixture_nav.header.sequence++;
  assert(RemoteId_SendLocation(1000)==PX4LITE_OK);
  mavlink_msg_open_drone_id_location_decode(&s_remoteid_message,&loc);assert(loc.timestamp==0.0f);
  fixture_nav.gnss_utc_sec=3599;assert(RemoteId_LocationTimestamp(&fixture_nav)==3599.0f);
  fixture_nav.gnss_utc_sec=3600;assert(RemoteId_LocationTimestamp(&fixture_nav)==0.0f);
  fixture_nav.valid_mask=PX4LITE_NAV_VALID_TIME;fixture_nav.header.sequence++;
  before=sends;assert(RemoteId_SendLocation(1000)==PX4LITE_NOT_READY && sends==before);
  assert(RemoteId_SendSystem(1000)==PX4LITE_NOT_READY);
  time_ready=1;fixture_time.header.valid=1;fixture_time.header.publish_time_ms=1000;
  fixture_time.source=PX4LITE_TIME_SOURCE_RTC;fixture_time.sync_state=PX4LITE_TIME_SYNC_RTC_VALID;
  fixture_time.utc_date_ymd=20190102;fixture_time.utc_time_hhmmss=0;
  assert(RemoteId_SendSystem(1000)==PX4LITE_OK);
  mavlink_msg_open_drone_id_system_decode(&s_remoteid_message,&system);assert(system.timestamp==86400);
  assert(RemoteId_SendSystem(3001)==PX4LITE_NOT_READY);
  fixture_time.utc_date_ymd=20190229;assert(RemoteId_SendSystem(1000)==PX4LITE_NOT_READY);
  puts("m2_remoteid_fields: PASS (actual MAVLink decode, independent valid bits, UTC/hour/epoch, speed/course)");return 0;
}
