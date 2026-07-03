/**
 * @file test_app_display_messagelog_local.c
 * @brief App_PushLocalMessageLog / App_CopyLocalMessageLog 本机日志读写。
 * @details
 * 替代已废弃 app_display_model.h 时代的同名旧测试：本机日志改由
 * App_PushLocalMessageLog() 写入、App_CopyLocalMessageLog() 读出，
 * 不再经过 App_MessageLogInit/Update 状态机或 App_DisplayLogSnapshot_t 包装。
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

/* app_data_api.c 单个编译单元内还引用了这些远端/本机数据源，本测试
   只驱动本机日志读写，其余一律 NOT_READY，仅满足链接。 */
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
  Px4Lite_LogEntry_t entries[8];
  uint16_t count;
  uint32_t version = 0U;
  uint16_t last_seq = 0U;

  App_ResetLocalMessageLog();
  App_PushLocalMessageLog(1U, 90101U, 0, 0, 0, 0U);
  App_PushLocalMessageLog(2U, 90102U, 0, 0, 0, 1U);

  memset(entries, 0, sizeof(entries));
  count = App_CopyLocalMessageLog(entries, 8U, &version, &last_seq);
  Eq("count", count, 2U);
  Eq("last_seq", last_seq, 2U);
  Eq("e0 mid", entries[0].message_id, 1U);
  Eq("e1 time", entries[1].time_hhmmss, 90102U);
  if (version == 0U) { printf("FAIL version not advanced\n"); g_fail++; }

  if (g_fail == 0) { printf("PASS test_app_display_messagelog_local\n"); return 0; }
  printf("FAIL test_app_display_messagelog_local fails=%d\n", g_fail);
  return 1;
}
