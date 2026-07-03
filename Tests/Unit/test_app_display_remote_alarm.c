/**
 * @file test_app_display_remote_alarm.c
 * @brief REMOTE 模式下 App_CopyRemoteDisplaySnapshot 正确带出远端最高告警。
 * @details
 * 替代已废弃 app_display_model.h 时代的 test_app_display_alarm_remote.c：
 * 远端不再下发完整告警表，只带最高故障码+来源+等级+活动位图，显示层从
 * App_CopyRemoteDisplaySnapshot() 的 alarms[0] 单条只读。
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

int main(void)
{
  uint32_t now = 5000U;
  Px4Lite_RemoteTelemetry_t snap;
  App_DisplaySnapshot_t out;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);

  memset(&snap, 0, sizeof(snap));
  snap.header.sequence        = 1U;
  snap.header.sample_time_ms  = now;
  snap.header.publish_time_ms = now;
  snap.header.valid           = 1U;
  snap.system_id              = TEST_REMOTE_SYSID;
  snap.highest_fault_code     = 0x10U;
  snap.highest_source_id      = 3U;
  snap.highest_severity       = (Px4Lite_AlarmSeverity_t)2U;
  snap.alarm_active_mask      = (1UL << 3) | (1UL << 7);
  snap.valid_mask            |= PX4LITE_REMOTE_VALID_ALARM;
  snap.alarm_update_ms        = now;
  (void)Px4Lite_RemoteTelemetryCommitNode(TEST_REMOTE_SYSID, &snap, now);

  memset(&out, 0, sizeof(out));
  ExpectU32("remote snapshot ok", App_CopyRemoteDisplaySnapshot(&out, now), 1U);
  ExpectU32("alarm valid", out.alarm_valid, 1U);
  ExpectU32("highest fault", out.highest_fault_code, 0x10U);
  ExpectU32("highest source", out.highest_source_id, 3U);
  ExpectU32("active count", out.alarm_active_count, 2U);
  ExpectU32("alarms0 src", out.alarms[0].source_id, 3U);
  ExpectU32("alarms0 code", out.alarms[0].fault_code, 0x10U);
  ExpectU32("alarms0 active", out.alarms[0].active, 1U);

  if (g_fail == 0) { printf("PASS test_app_display_remote_alarm\n"); return 0; }
  printf("FAIL test_app_display_remote_alarm fails=%d\n", g_fail);
  return 1;
}
