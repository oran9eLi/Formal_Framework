/** @file test_gnss_tx_freshness.c
 * @brief 真实 GPS 和 GLOBAL_POSITION 编码不对已过期字段作有效声明。
 */
#include <assert.h>
#include <stdio.h>
#include "../../Framework/Src/px4lite_mavlink_tx.c"
static Px4Lite_SensorGnss_t gnss;
static Px4Lite_VehicleNavigation_t nav;
static Px4Lite_BatteryStatus_t battery_source;
Px4Lite_Result_t Px4Lite_CopyGnss(Px4Lite_SensorGnss_t *out) { *out=gnss; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { *out=nav; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { *out=battery_source; return PX4LITE_OK; }
uint8_t Px4Lite_IdentityGetMavlinkSystemId(void) { return 42U; }
Px4Lite_Result_t Px4Lite_CommSend(const uint8_t *data, uint16_t len) { (void)data; (void)len; return PX4LITE_OK; }
Px4Lite_Result_t Px4Lite_RpiMavlinkSend(const uint8_t *data, uint16_t len) { (void)data; (void)len; return PX4LITE_OK; }
int main(void)
{
  mavlink_gps_raw_int_t wire;
  mavlink_battery_status_t battery_wire;
  gnss.header.valid=1U; gnss.header.sequence=1U; gnss.header.sample_time_ms=2000U;
  gnss.fix_type=1U; gnss.fix_dimension=3U; gnss.position_update_ms=2000U; gnss.altitude_update_ms=2000U;
  gnss.valid_mask=PX4LITE_GNSS_VALID_POSITION|PX4LITE_GNSS_VALID_ALTITUDE|PX4LITE_GNSS_VALID_VELOCITY;
  gnss.ground_speed_cms=123U; gnss.heading_deg100=9000U; gnss.velocity_update_ms=0U;
  assert(MavTx_SendGpsRaw(2000U)==PX4LITE_OK);
  mavlink_msg_gps_raw_int_decode(&s_message,&wire);
  assert(wire.vel==UINT16_MAX && wire.cog==UINT16_MAX);
  nav.header.valid=1U; nav.header.sample_time_ms=2000U; nav.valid_mask=PX4LITE_NAV_VALID_POSITION;
  assert(MavTx_SendPosition(2000U)==PX4LITE_NOT_READY);
  battery_source.header.valid=1U; battery_source.header.sequence=1U; battery_source.header.sample_time_ms=2000U;
  battery_source.voltage_mv=12000U; battery_source.current_ma=0;
  assert(MavTx_SendBatteryStatus(2000U)==PX4LITE_OK);
  mavlink_msg_battery_status_decode(&s_message,&battery_wire);
  assert(battery_wire.current_battery==-1 && battery_wire.voltages[0]==12000U);
  battery_source.current_valid=1U; battery_source.header.sequence++;
  assert(MavTx_SendBatteryStatus(2000U)==PX4LITE_OK);
  mavlink_msg_battery_status_decode(&s_message,&battery_wire);
  assert(battery_wire.current_battery==0 && battery_wire.voltages[0]==12000U);
  puts("gnss transmit freshness tests passed"); return 0;
}
