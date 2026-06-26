/**
 * @file test_app_display_alarm_remote.c
 * @brief REMOTE 模式 App_GetDisplayAlarm 返回完整远端告警行。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_display_model.h"
#include "px4lite_remote_telemetry.h"

static int g_fail;
static Px4Lite_RemoteMode_t s_mode;

Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return s_mode; }

/* 远端取数委托给真实 RemoteTelemetry，测试用 Mutable+Commit 布置快照。 */
Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{
  return Px4Lite_RemoteTelemetryCopySnapshot(out, now_ms);
}

Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *out, uint32_t now_ms) { (void)out; (void)now_ms; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *o, uint32_t n) { (void)o; (void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *o, uint32_t n) { (void)o; (void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *o, uint32_t n) { (void)o; (void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *o, uint32_t n) { (void)o; (void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *o, uint32_t n) { (void)o; (void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t m, Px4Lite_ModuleStatus_t *o) { (void)m; (void)o; return PX4LITE_NOT_READY; }
void App_GetCommStats(Px4Lite_CommDebugInfo_t *o) { if (o) { memset(o, 0, sizeof(*o)); } }

static void ExpectU32(const char *name, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", name, (unsigned long)a, (unsigned long)e); g_fail++; } }

static void TestRemoteReturnsFullTable(void)
{
  Px4Lite_RemoteTelemetrySnapshot_t *snap;
  App_AlarmSnapshot_t out;
  uint32_t now = 5000U;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  s_mode = PX4LITE_REMOTE_MODE_REMOTE;

  snap = Px4Lite_RemoteTelemetryMutable();
  memset(snap->alarm_records, 0, sizeof(snap->alarm_records));
  snap->alarm_records[0].source_id = 3U; snap->alarm_records[0].fault_code = 0x10U; snap->alarm_records[0].severity = (Px4Lite_AlarmSeverity_t)2U; snap->alarm_records[0].active = 1U;
  snap->alarm_records[1].source_id = 7U; snap->alarm_records[1].fault_code = 0x22U; snap->alarm_records[1].severity = (Px4Lite_AlarmSeverity_t)1U; snap->alarm_records[1].active = 1U;
  snap->alarm_table_count = 2U;
  snap->highest_fault_code = 0x10U; snap->highest_source_id = 3U; snap->highest_severity = 2U;
  Px4Lite_RemoteTelemetryCommit(2U, 1U, PX4LITE_REMOTE_VALID_ALARM, now);

  memset(&out, 0, sizeof(out));
  ExpectU32("remote alarm ok", App_GetDisplayAlarm(&out, now), PX4LITE_OK);
  ExpectU32("active_count", out.active_count, 2U);
  ExpectU32("rec0 src", out.records[0].source_id, 3U);
  ExpectU32("rec1 src", out.records[1].source_id, 7U);
  ExpectU32("rec1 code", out.records[1].fault_code, 0x22U);
}

int main(void)
{
  TestRemoteReturnsFullTable();
  if (g_fail == 0) { printf("PASS test_app_display_alarm_remote\n"); return 0; }
  printf("FAIL test_app_display_alarm_remote fails=%d\n", g_fail);
  return 1;
}
