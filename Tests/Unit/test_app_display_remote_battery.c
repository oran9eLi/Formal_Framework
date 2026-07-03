/**
 * @file test_app_display_remote_battery.c
 * @brief REMOTE 模式下 App_CopyRemoteDisplaySnapshot 正确带出主控/电机双电池字段。
 * @details
 * 替代已废弃 app_display_model.h 时代的 test_app_display_model_remote_power.c：
 * 显示模型早已收口进 app_data_api.h，远端电压电量改由
 * App_CopyRemoteDisplaySnapshot() 提供。这里同时覆盖一个曾经排查过的边界：
 * 电池模块状态必须是 ONLINE/DEGRADED 才会保留电压电量读数，否则会被清零
 * (App_ViewStateHasUsableData 门槛)——用两组数据分别验证"模块在线时保留
 * 读数"和"模块状态未知时清零读数"两条路径。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_data_api.h"
#include "px4lite_alarm.h"
#include "px4lite_control.h"
#include "px4lite_local_msglog.h"
#include "px4lite_mavlink_rx.h"
#include "px4lite_mavlink_tx.h"
#include "px4lite_modules.h"
#include "px4lite_remote_telemetry.h"
#include "px4lite_time.h"

static int g_fail;
static void ExpectU32(const char *name, uint32_t actual, uint32_t expected)
{ if (actual != expected) { printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)actual, (unsigned long)expected); g_fail++; } }

#define TEST_REMOTE_SYSID 3U

/* app_data_api.c 依赖的 Framework 数据源桩：本测试只驱动远端遥测路径，
   本机相关 Copy* 一律 NOT_READY。 */
Px4Lite_Result_t Px4Lite_CopyNavigation(Px4Lite_VehicleNavigation_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBaro(Px4Lite_SensorBaro_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyBattery2(Px4Lite_BatteryStatus_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyMotor(Px4Lite_MotorOutputs_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyHealth(Px4Lite_SystemHealth_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyTime(Px4Lite_TimeSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSnapshot(Px4Lite_AlarmSnapshot_t *out) { (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmSummary(uint32_t *publish_time_ms, uint32_t *sequence, uint16_t *active_count, uint16_t *highest_fault_code, uint16_t *highest_source_id, Px4Lite_AlarmSeverity_t *highest_severity)
{ (void)publish_time_ms; (void)sequence; (void)active_count; (void)highest_fault_code; (void)highest_source_id; (void)highest_severity; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyAlarmRecord(uint16_t index, Px4Lite_AlarmRecord_t *out) { (void)index; (void)out; return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_CopyModuleStatuses(Px4Lite_ModuleStatus_t *status, uint16_t count, uint32_t *version) { (void)status; (void)count; if (version) { *version = 0U; } return PX4LITE_NOT_READY; }
Px4Lite_Result_t Px4Lite_GetModuleStatus(Px4Lite_ModuleId_t module_id, Px4Lite_ModuleStatus_t *status) { (void)module_id; (void)status; return PX4LITE_NOT_READY; }
uint32_t Px4Lite_GetStatusVersion(void) { return 0U; }
void Px4Lite_GetCommDebugInfo(Px4Lite_CommDebugInfo_t *out) { if (out) { memset(out, 0, sizeof(*out)); } }
Px4Lite_Result_t Px4Lite_ControlSetMotorThrottlePercent(uint8_t motor_index, uint8_t throttle_percent) { (void)motor_index; (void)throttle_percent; return PX4LITE_NOT_READY; }
void Px4Lite_MavlinkSetTxEnabled(uint8_t enabled) { (void)enabled; }
Px4Lite_Result_t Px4Lite_CopyCommRxFrame(Px4Lite_CommRxFrame_t *out) { (void)out; return PX4LITE_NOT_READY; }

static void CommitRemoteBattery(uint32_t now_ms, uint8_t battery_module_online)
{
  Px4Lite_RemoteTelemetry_t snap;

  memset(&snap, 0, sizeof(snap));
  snap.header.sequence        = 1U;
  snap.header.sample_time_ms  = now_ms;
  snap.header.publish_time_ms = now_ms;
  snap.header.valid           = 1U;
  snap.system_id              = TEST_REMOTE_SYSID;

  snap.voltage_mv        = 11800U;
  snap.battery_percent   = 86U;
  snap.voltage2_mv       = 12050U;
  snap.battery2_percent  = 91U;
  snap.low_voltage2      = 1U;
  snap.valid_mask       |= PX4LITE_REMOTE_VALID_BATTERY;
  snap.battery_update_ms = now_ms;

  /* 已收到过模块状态帧(MODULES 位)，但电池模块的具体状态位是否在其中，
     取决于 battery_module_online：这正是曾经排查过的那个真实 bug 场景——
     模块状态帧到了，但没带电池模块的位，battery.state 停留在 UNKNOWN，
     会被判定为"不可用"从而清零电压电量。 */
  snap.valid_mask     |= PX4LITE_REMOTE_VALID_MODULES;
  snap.modules_update_ms = now_ms;
  if (battery_module_online != 0U) {
    snap.module_state[PX4LITE_MODULE_BATTERY]  = PX4LITE_STATE_ONLINE;
    snap.module_state_valid_mask               |= (1UL << (uint32_t)PX4LITE_MODULE_BATTERY);
  }

  (void)Px4Lite_RemoteTelemetryCommitNode(TEST_REMOTE_SYSID, &snap, now_ms);
}

static void TestBatteryKeptWhenModuleOnline(void)
{
  App_DisplaySnapshot_t env;
  uint32_t now = 1000U;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);
  CommitRemoteBattery(now, 1U);

  memset(&env, 0, sizeof(env));
  ExpectU32("remote snapshot ok", App_CopyRemoteDisplaySnapshot(&env, now), 1U);
  ExpectU32("main voltage", env.voltage_mv, 11800U);
  ExpectU32("main percent", env.battery_percent, 86U);
  ExpectU32("motor voltage", env.voltage2_mv, 12050U);
  ExpectU32("motor percent", env.battery2_percent, 91U);
  ExpectU32("motor low voltage", env.low_voltage2, 1U);
}

static void TestBatteryClearedWhenModuleUnknown(void)
{
  App_DisplaySnapshot_t env;
  uint32_t now = 2000U;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);
  CommitRemoteBattery(now, 0U);

  memset(&env, 0, sizeof(env));
  ExpectU32("remote snapshot ok (module unknown)", App_CopyRemoteDisplaySnapshot(&env, now), 1U);
  ExpectU32("main voltage cleared", env.voltage_mv, 0U);
  ExpectU32("main percent cleared", env.battery_percent, 0U);
}

int main(void)
{
  TestBatteryKeptWhenModuleOnline();
  TestBatteryClearedWhenModuleUnknown();

  if (g_fail == 0) { printf("PASS test_app_display_remote_battery\n"); return 0; }
  printf("FAIL test_app_display_remote_battery fails=%d\n", g_fail);
  return 1;
}
