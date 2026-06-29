/**
 * @file test_app_display_messagelog_local.c
 * @brief LOCAL 模式 App_GetDisplayMessageLog 返回业务日志缓冲。
 */
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "app_display_model.h"
#include "app_message_log.h"
#include "app_data_api.h"

static int g_fail;
static void Eq(const char *n, uint32_t a, uint32_t e)
{ if (a != e) { printf("FAIL %s actual=%lu expected=%lu\n", n, (unsigned long)a, (unsigned long)e); g_fail++; } }

/* 模式固定为 LOCAL；远端取数本测试不触发，桩为 NOT_READY 仅满足链接。 */
Px4Lite_RemoteMode_t App_GetRemoteDisplayMode(void) { return PX4LITE_REMOTE_MODE_LOCAL; }
Px4Lite_Result_t App_CopyRemoteTelemetry(Px4Lite_RemoteTelemetrySnapshot_t *out, uint32_t now_ms)
{ (void)now_ms; if (out) { memset(out, 0, sizeof(*out)); } return PX4LITE_NOT_READY; }

/* 数据源桩：全在线，电压正常，无告警。 */
Px4Lite_Result_t App_GetModuleStatus(Px4Lite_ModuleId_t m, Px4Lite_ModuleStatus_t *o)
{ if (o == 0) { return PX4LITE_INVALID_PARAM; } memset(o, 0, sizeof(*o)); o->module_id = m; o->state = PX4LITE_STATE_ONLINE; o->last_rx_ms = 1U; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyEnvironment(App_EnvironmentSnapshot_t *o, uint32_t n)
{ (void)n; if (o == 0) { return PX4LITE_INVALID_PARAM; } memset(o, 0, sizeof(*o)); o->voltage_mv = 12000U; o->voltage2_mv = 12000U; return PX4LITE_OK; }
Px4Lite_Result_t App_CopyAlarm(App_AlarmSnapshot_t *o, uint32_t n)
{ (void)n; if (o == 0) { return PX4LITE_INVALID_PARAM; } memset(o, 0, sizeof(*o)); return PX4LITE_OK; }
/* 其余 getter 用到的 App_Copy* 桩空(本测试不触发它们)。 */
Px4Lite_Result_t App_CopyNavigation(App_NavigationSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopySystem(App_SystemSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyMotor(App_MotorSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
Px4Lite_Result_t App_CopyDateTime(App_DateTimeSnapshot_t *o, uint32_t n) { (void)o;(void)n; return PX4LITE_NOT_READY; }
void App_GetCommStats(Px4Lite_CommDebugInfo_t *o) { if (o) { memset(o, 0, sizeof(*o)); } }

int main(void)
{
  App_DisplayLogSnapshot_t out;
  uint32_t now = 1000U;
  uint32_t end;

  App_MessageLogInit(now);
  App_MessageLogUpdate(now);
  end = now + 2500U;
  while (now < end) { now += 100U; App_MessageLogUpdate(now); }

  Eq("local log ok", App_GetDisplayMessageLog(&out, now), PX4LITE_OK);
  if (out.count == 0U) { printf("FAIL empty local log\n"); g_fail++; }
  Eq("first system_start", out.entries[0].message_id, APP_LOGMSG_SYSTEM_START);

  if (g_fail == 0) { printf("PASS test_app_display_messagelog_local\n"); return 0; }
  printf("FAIL test_app_display_messagelog_local fails=%d\n", g_fail); return 1;
}
