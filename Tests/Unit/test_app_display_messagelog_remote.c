/**
 * @file test_app_display_messagelog_remote.c
 * @brief REMOTE 模式下 App_CopyRemoteMessageLog 正确带出远端消息日志。
 * @details
 * 替代已废弃 app_display_model.h 时代的同名旧测试：显示模型收口进
 * app_data_api.h 后，远端日志改由 App_CopyRemoteMessageLog() 提供，
 * 不再依赖 App_GetDisplayMessageLog / App_DisplayLogSnapshot_t。
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
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

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
  Px4Lite_LogEntry_t entries[8];
  uint16_t count;
  uint16_t last_seq = 0U;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  (void)Px4Lite_SelectRemoteNode(TEST_REMOTE_SYSID);

  memset(&snap, 0, sizeof(snap));
  snap.header.sequence        = 1U;
  snap.header.sample_time_ms  = now;
  snap.header.publish_time_ms = now;
  snap.header.valid           = 1U;
  snap.system_id              = TEST_REMOTE_SYSID;
  snap.remote_log_entries[0].sequence = 5U; snap.remote_log_entries[0].message_id = 4U; snap.remote_log_entries[0].time_hhmmss = 111U;
  snap.remote_log_entries[1].sequence = 6U; snap.remote_log_entries[1].message_id = 12U; snap.remote_log_entries[1].time_hhmmss = 222U;
  snap.remote_log_count       = 2U;
  snap.remote_log_latest_seq  = 6U;
  snap.valid_mask            |= PX4LITE_REMOTE_VALID_LOG;
  snap.log_update_ms          = now;
  (void)Px4Lite_RemoteTelemetryCommitNode(TEST_REMOTE_SYSID, &snap, now);

  memset(entries, 0, sizeof(entries));
  count = App_CopyRemoteMessageLog(entries, 8U, &last_seq, now);
  Eq("count", count, 2U);
  Eq("last_seq", last_seq, 6U);
  Eq("e0 seq", entries[0].sequence, 5U);
  Eq("e1 mid", entries[1].message_id, 12U);

  if (g_fail == 0) { printf("PASS test_app_display_messagelog_remote\n"); return 0; }
  printf("FAIL test_app_display_messagelog_remote fails=%d\n", g_fail);
  return 1;
}
