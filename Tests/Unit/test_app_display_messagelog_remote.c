/**
 * @file test_app_display_messagelog_remote.c
 * @brief REMOTE 模式 App_GetDisplayMessageLog 返回远端日志。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_display_model.h"
#include "app_message_log.h"
#include "px4lite_remote_telemetry.h"

static int g_fail;
static Px4Lite_RemoteMode_t s_mode;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return s_mode; }
Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{ return Px4Lite_RemoteTelemetryCopySnapshot(out, now_ms); }
/* 其余 App_Copy* / 数据源桩(本测试不触发)。 */
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t m, Px4Lite_ModuleStatus_t *o) { (void)m; if (o) memset(o,0,sizeof(*o)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *o, uint32_t n) { (void)n; if (o) memset(o,0,sizeof(*o)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *o, uint32_t n) { (void)n; if (o) memset(o,0,sizeof(*o)); return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
void App_GetCommStats(Px4Lite_CommDebugInfo_t *o) { if (o) memset(o,0,sizeof(*o)); }

int main(void)
{
  Px4Lite_RemoteTelemetrySnapshot_t *snap;
  App_DisplayLogSnapshot_t out;
  uint32_t now = 5000U;

  Px4Lite_RemoteTelemetryInit(now);
  (void)Px4Lite_RemoteTelemetrySetMode(PX4LITE_REMOTE_MODE_REMOTE, now);
  s_mode = PX4LITE_REMOTE_MODE_REMOTE;

  snap = Px4Lite_RemoteTelemetryMutable();
  memset(snap->log_entries, 0, sizeof(snap->log_entries));
  snap->log_entries[0].sequence = 5U; snap->log_entries[0].message_id = 4U; snap->log_entries[0].time_hhmmss = 111U;
  snap->log_entries[1].sequence = 6U; snap->log_entries[1].message_id = 12U; snap->log_entries[1].time_hhmmss = 222U;
  snap->log_count = 2U; snap->log_last_seq = 6U;
  Px4Lite_RemoteTelemetryCommit(2U, 1U, PX4LITE_REMOTE_VALID_LOG, now);

  Eq("remote log ok", App_GetDisplayMessageLog(&out, now), PX4LITE_OK);
  Eq("count", out.count, 2U);
  Eq("e0 mid", out.entries[0].message_id, 4U);
  Eq("e1 seq", out.entries[1].sequence, 6U);
  Eq("version=last_seq", out.version, 6U);

  if (g_fail == 0) { printf("PASS test_app_display_messagelog_remote\n"); return 0; }
  printf("FAIL test_app_display_messagelog_remote fails=%d\n", g_fail); return 1;
}
